"""
c2demo-wakeup Lambda — start/stop/status for C2 Demo EC2 instance.
Routes: GET /status  POST /start  POST /stop
"""
import json
import boto3

INSTANCE_ID = 'i-02dddc042030e4557'
REGION      = 'us-west-2'

ec2 = boto3.client('ec2', region_name=REGION)

CORS = {
    'Access-Control-Allow-Origin':  '*',
    'Access-Control-Allow-Methods': 'GET,POST,OPTIONS',
    'Access-Control-Allow-Headers': 'Content-Type',
    'Content-Type': 'application/json',
}

def reply(data, code=200):
    return {'statusCode': code, 'headers': CORS, 'body': json.dumps(data)}

def describe():
    r = ec2.describe_instances(InstanceIds=[INSTANCE_ID])
    return r['Reservations'][0]['Instances'][0]

def lambda_handler(event, context):
    method = event.get('requestContext', {}).get('http', {}).get('method', 'GET')
    path   = event.get('rawPath', '/status')

    # CORS preflight
    if method == 'OPTIONS':
        return reply({})

    if path == '/start' and method == 'POST':
        inst  = describe()
        state = inst['State']['Name']
        ip    = inst.get('PublicIpAddress')

        if state == 'stopped':
            ec2.start_instances(InstanceIds=[INSTANCE_ID])
            return reply({'state': 'starting', 'ip': None})

        return reply({'state': state, 'ip': ip})

    if path == '/stop' and method == 'POST':
        ec2.stop_instances(InstanceIds=[INSTANCE_ID])
        return reply({'state': 'stopping'})

    # Default: /status
    inst  = describe()
    state = inst['State']['Name']
    ip    = inst.get('PublicIpAddress')
    return reply({'state': state, 'ip': ip})
