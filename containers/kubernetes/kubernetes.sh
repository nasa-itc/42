OU=myorg
ENVIRO=dev
CONTEXT=missions
REALM=${OU}-${ENVIRO}-${CONTEXT}

MISSION=m01
SPACECRAFT=sc01
# Timeouts for starting the cluster
TIMEOUT_SECONDS=300 # 5 minutes
SLEEP_INTERVAL=15    # Wait 15 seconds between checks

# resource names, to be used with context, cluster, namespace, pods, and services
RESOURCE_NAME=${REALM}-${MISSION}

# application specifics
APP=42
APP_INTERNAL_PORT=80
APP_EXTERNAL_PORT=30080
IMAGE_URI=ghcr.io/ericstoneking/42:latest 

# contexts/clusters
K8S_CONTEXT=${RESOURCE_NAME}
K8S_CLUSTER=${K8S_CONTEXT}

# 
KIND_EXPERIMENTAL_PROVIDER=podman kind delete cluster --name ${K8S_CLUSTER} || true
KIND_EXPERIMENTAL_PROVIDER=podman kind create cluster --name ${K8S_CLUSTER}

# add the prefix since podman adds kind- to contexts/clusters
K8S_CONTEXT_PREFIX=kind-
K8S_CONTEXT=${K8S_CONTEXT_PREFIX}${K8S_CONTEXT}

# set and use context
kubectl config set-context ${K8S_CONTEXT}
kubectl config use-context ${K8S_CONTEXT}

# namespace
K8S_NAMESPACE=${RESOURCE_NAME}-${SPACECRAFT}
K8S_POD=${RESOURCE_NAME}-${SPACECRAFT}-${APP}

kubectl delete namespace ${K8S_NAMESPACE} --context ${K8S_CONTEXT} || true
kubectl create namespace ${K8S_NAMESPACE} --context ${K8S_CONTEXT}

# deployment
K8S_DEPLOYMENT=${K8S_POD}

kubectl delete deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} || true
kubectl create deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} \
  --port=${APP_INTERNAL_PORT} \
  --image=${IMAGE_URI} \
  --replicas=1

echo "Waiting for pod in namespace ${K8S_NAMESPACE} to be in a 'Running' state..."

start_time=$(date +%s)
while true; do
    # Use kubectl with jq to find the pod and get its status and name
    # We select the pod whose name starts with our prefix
    pod_info=$(kubectl get pods -n "$K8S_NAMESPACE" -o json 2>/dev/null | \
               jq -r '.items[] | select(.metadata.name | startswith("'$K8S_NAMESPACE'")) | {name: .metadata.name, status: .status.phase}')

    # Check if a pod was found and get its name and status
    if [ -n "$pod_info" ]; then
        POD_NAME=$(echo "$pod_info" | jq -r '.name')
        pod_status=$(echo "$pod_info" | jq -r '.status')
    else
        POD_NAME=""
        pod_status=""
    fi

    # Check if the pod is in the "Running" status
    if [ "$pod_status" == "Running" ]; then
        echo "Success: Pod '$POD_NAME' is now running."
        break # Exit the loop
    fi

    # Check for timeout
    current_time=$(date +%s)
    elapsed_time=$((current_time - start_time))
    if [ "$elapsed_time" -ge "$TIMEOUT_SECONDS" ]; then
        if [ -z "$POD_NAME" ]; then
            echo "Error: Timeout reached ($TIMEOUT_SECONDS seconds). No pod with prefix '$POD_NAME_PREFIX' was found."
        else
            echo "Error: Timeout reached ($TIMEOUT_SECONDS seconds). Pod '$POD_NAME' status is '$pod_status'."
        fi
        exit 1
    fi

    # Report current status and wait before the next check
    if [ -n "$POD_NAME" ]; then
        echo "Current status for '$POD_NAME': '$pod_status'. Waiting..."
    else
        echo "No pod with prefix '$POD_NAME_PREFIX' found yet. Waiting..."
    fi
    sleep "$SLEEP_INTERVAL"
done

kubectl scale deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} --replicas=1

# service
K8S_SERVICE=${K8S_DEPLOYMENT}-service

kubectl delete service ${K8S_SERVICE} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} || true
kubectl expose deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} \
  --name=${K8S_SERVICE} \
  --port=${APP_INTERNAL_PORT} \
  --target-port=${APP_INTERNAL_PORT} \
  --protocol=TCP

# port-forwarding
kubectl port-forward service/${K8S_SERVICE} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} \
  ${APP_EXTERNAL_PORT}:${APP_INTERNAL_PORT} &

echo "open browser to http://localhost:${APP_EXTERNAL_PORT}/vnc.html and click on 'Connect'"
