#! /bin/bash
set -euo pipefail

## Upload a deployment
## from the "com.github.stephengold" namespace in Sonatype's OSSRH staging area
## to Sonatype's Central Publisher Portal
## so the deployment can be tested and then published or dropped.

## IMPORTANT:  The upload request must originate
## from the IP address used to stage the deployment to the staging area!

# The required -p and -u flags on the command line
# specify the password and username components of a "user token"
# generated using the web interface at https://central.sonatype.com/account

while getopts p:u: flag
do
    case "${flag}" in
        p) centralPassword=${OPTARG};;
        u) centralUsername=${OPTARG};;
        *) echo "Unknown argument: ${flag}" ; exit 1
    esac
done

# Combine both components into a base64 "user token"
# suitable for the Authorization header of a POST request:

token=$(printf %s:%s "${centralUsername}" "${centralPassword}" | base64)

# Send a POST request to upload the deployment:

server='ossrh-staging-api.central.sonatype.com'
endpoint='/manual/upload/defaultRepository/com.github.stephengold'
url="https://${server}${endpoint}"

statusCode=$(curl "${url}" \
  --no-progress-meter \
  --output postData1.txt \
  --write-out '%{response_code}' \
  --request POST \
  --header 'accept: */*' \
  --header "Authorization: Bearer ${token}" \
  --data '')

echo "Status code = ${statusCode}"
echo 'Received data:'
cat postData1.txt
echo '[EOF]'

# Retry if the default repository isn't found (status=400).

if [ "${statusCode}" == "400" ]; then
  echo "Will retry after 90 seconds."
  sleep 90

  statusCode2=$(curl "${url}" \
    --no-progress-meter \
    --output postData2.txt \
    --write-out '%{response_code}' \
    --request POST \
    --header 'accept: */*' \
    --header "Authorization: Bearer ${token}" \
    --data '')

  echo "Status code = ${statusCode2}"
  echo 'Received data:'
  cat postData2.txt
  echo '[EOF]'
fi
