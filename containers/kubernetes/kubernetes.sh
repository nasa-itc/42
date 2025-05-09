OU=myorg
ENVIRO=dev
CONTEXT=missions
REALM=${OU}-${ENVIRO}-${CONTEXT}

MISSION=m01
SPACECRAFT=sc01

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
K8S_CONTEXT_PRIFIX=kind-
K8S_CONTEXT=${K8S_CONTEXT_PRIFIX}${K8S_CONTEXT}

# set and use context
kubectl config set-context ${K8S_CONTEXT}
kubectl config use-context ${K8S_CONTEXT}

# namespace
K8S_NAMESPACE=${RESOURCE_NAME}-${SPACECRAFT}
K8S_POD=${RESOURCE_NAME}-${SPACECRAFT}-${APP}

kubectl delete namespace ${K8S_NAMESPACE} --context ${K8S_CONTEXT} || true
kubectl create namespace ${K8S_NAMESPACE} --context ${K8S_CONTEXT}

# deployment
K8S_DEPLOYMENT=${K8S_NAMESPACE}

kubectl delete deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} || true
kubectl create deployment ${K8S_DEPLOYMENT} --context ${K8S_CONTEXT} --namespace ${K8S_NAMESPACE} \
  --port=${APP_INTERNAL_PORT} \
  --image=${IMAGE_URI} \
  --replicas=1

echo "Sleeping for 120 seconds to ensure that pod(s) are up and running"
sleep 120

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
