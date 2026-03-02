#!/usr/bin/env bash
# deploy.sh — one-shot deploy of C2 Demo wake-up Lambda + API Gateway
# Prerequisites: AWS CLI configured (aws configure), region us-west-2
# Run once. Re-running is safe (updates existing resources).
set -euo pipefail

REGION="us-west-2"
FUNCTION="c2demo-wakeup"
ROLE="c2demo-lambda-role"
API_NAME="c2demo-api"
DIR="$(cd "$(dirname "$0")" && pwd)"

echo "[1/7] Getting AWS account ID..."
ACCOUNT=$(aws sts get-caller-identity --query Account --output text)
echo "      Account: $ACCOUNT"

# ── IAM role ──────────────────────────────────────────────────────────────
echo "[2/7] Creating IAM role..."
TRUST='{
  "Version":"2012-10-17",
  "Statement":[{"Effect":"Allow","Principal":{"Service":"lambda.amazonaws.com"},"Action":"sts:AssumeRole"}]
}'

ROLE_ARN=$(aws iam create-role \
  --role-name "$ROLE" \
  --assume-role-policy-document "$TRUST" \
  --query 'Role.Arn' --output text 2>/dev/null) \
|| ROLE_ARN=$(aws iam get-role --role-name "$ROLE" --query 'Role.Arn' --output text)

echo "      Role ARN: $ROLE_ARN"

# Inline policy: only the permissions actually needed
POLICY='{
  "Version":"2012-10-17",
  "Statement":[
    {"Effect":"Allow","Action":["ec2:StartInstances","ec2:StopInstances","ec2:DescribeInstances"],"Resource":"*"},
    {"Effect":"Allow","Action":["logs:CreateLogGroup","logs:CreateLogStream","logs:PutLogEvents"],"Resource":"arn:aws:logs:*:*:*"}
  ]
}'
aws iam put-role-policy --role-name "$ROLE" --policy-name c2demo-ec2 --policy-document "$POLICY"
echo "      Policy attached."
sleep 8   # IAM propagation delay

# ── Package Lambda ────────────────────────────────────────────────────────
echo "[3/7] Packaging Lambda..."
cd "$DIR"
zip -q lambda.zip lambda_function.py
echo "      lambda.zip ready."

# ── Create / update Lambda function ──────────────────────────────────────
echo "[4/7] Deploying Lambda function..."
LAMBDA_ARN=$(aws lambda create-function \
  --function-name "$FUNCTION" \
  --runtime python3.12 \
  --role "$ROLE_ARN" \
  --handler lambda_function.lambda_handler \
  --zip-file fileb://lambda.zip \
  --timeout 15 \
  --region "$REGION" \
  --query 'FunctionArn' --output text 2>/dev/null) \
|| {
  aws lambda update-function-code \
    --function-name "$FUNCTION" \
    --zip-file fileb://lambda.zip \
    --region "$REGION" > /dev/null
  LAMBDA_ARN="arn:aws:lambda:${REGION}:${ACCOUNT}:function:${FUNCTION}"
}
echo "      Lambda ARN: $LAMBDA_ARN"
rm lambda.zip

# ── Create / reuse API Gateway HTTP API ──────────────────────────────────
echo "[5/7] Creating API Gateway HTTP API..."
API_ID=$(aws apigatewayv2 create-api \
  --name "$API_NAME" \
  --protocol-type HTTP \
  --cors-configuration \
    AllowOrigins='["*"]',AllowMethods='["GET","POST","OPTIONS"]',AllowHeaders='["*"]' \
  --region "$REGION" \
  --query 'ApiId' --output text 2>/dev/null) \
|| API_ID=$(aws apigatewayv2 get-apis --region "$REGION" \
    --query "Items[?Name=='${API_NAME}'].ApiId" --output text)
echo "      API ID: $API_ID"

# ── Lambda integration ────────────────────────────────────────────────────
echo "[6/7] Creating Lambda integration..."
INTEG_ID=$(aws apigatewayv2 create-integration \
  --api-id "$API_ID" \
  --integration-type AWS_PROXY \
  --integration-uri "$LAMBDA_ARN" \
  --payload-format-version 2.0 \
  --region "$REGION" \
  --query 'IntegrationId' --output text)

# Route all requests to Lambda ($default catch-all)
aws apigatewayv2 create-route \
  --api-id "$API_ID" \
  --route-key '$default' \
  --target "integrations/$INTEG_ID" \
  --region "$REGION" > /dev/null

# Deploy stage
aws apigatewayv2 create-stage \
  --api-id "$API_ID" \
  --stage-name '$default' \
  --auto-deploy \
  --region "$REGION" > /dev/null 2>&1 || true

# Grant API Gateway permission to invoke Lambda
aws lambda add-permission \
  --function-name "$FUNCTION" \
  --statement-id apigw-invoke \
  --action lambda:InvokeFunction \
  --principal apigateway.amazonaws.com \
  --source-arn "arn:aws:execute-api:${REGION}:${ACCOUNT}:${API_ID}/*" \
  --region "$REGION" > /dev/null 2>&1 || true

# ── Output ────────────────────────────────────────────────────────────────
API_URL="https://${API_ID}.execute-api.${REGION}.amazonaws.com"
echo ""
echo "[7/7] Done! API URL:"
echo "      $API_URL"
echo ""
echo "  Test it:"
echo "    curl ${API_URL}/status"
echo "    curl -X POST ${API_URL}/start"
echo ""
echo "  Now update API_URL in launch/index.html with:"
echo "    $API_URL"
echo ""

# Auto-patch index.html with the real API URL
sed -i "s|https://REPLACE_WITH_API_URL|${API_URL}|g" "${DIR}/index.html"
echo "  index.html patched with API URL automatically."
