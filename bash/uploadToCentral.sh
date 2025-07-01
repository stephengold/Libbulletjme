#! /bin/bash
set -euo pipefail

## Upload the default repository for com.github.stephengold
## (staged using the Portal OSSRH Staging API)
## to the Central Publisher Portal
## so it can be tested and then published or dropped.

## IMPORTANT:  The upload request must originate
## from the IP address used to stage the deployment!

# The required -p and -u flags on the command line
# specify the password and username components of a "user token"
# generated using the web interface at https://central.sonatype.com/account

while getopts p:u: flag
do
    case "${flag}" in
        p) centralPassword=${OPTARG};;
        u) centralUsername=${OPTARG};;
    esac
done

# Combine both components into a base64 "user token"
# suitable for the Authorization header of a POST request:

token=$(printf %s:%s "${centralUsername}" "${centralPassword}" | base64)

# Send a POST request to upload the repository:

curl --include --request POST \
  'https://ossrh-staging-api.central.sonatype.com/manual/upload/defaultRepository/com.github.stephengold' \
  --header 'accept: */*' \
  --header "Authorization: Bearer $token" \
  --data ''
