#!/bin/bash
# ViDeG EC2 nginx + WSS setup
# Run this ON EC2 after SSH-ing in:
#   bash setup_ec2_nginx.sh YOUR_DOMAIN YOUR_EMAIL
#
# Pre-requisites:
#   1. Domain DNS A record already pointing to this EC2 IP (44.253.251.75)
#   2. EC2 security group has ports 80 and 443 open (can close 8765/8766 after this)
#   3. videg_relay.py already copied to ~/

set -e

DOMAIN=${1:?Usage: $0 YOUR_DOMAIN YOUR_EMAIL}
EMAIL=${2:?Usage: $0 YOUR_DOMAIN YOUR_EMAIL}

echo "=== ViDeG nginx + WSS setup for $DOMAIN ==="

# ── Install nginx + certbot ──────────────────────────────────────────────────
sudo apt update
sudo apt install -y nginx certbot python3-certbot-nginx

# ── Web root for console.html ────────────────────────────────────────────────
mkdir -p ~/videg-web
echo "Web root: ~/videg-web  (upload console.html here)"

# ── nginx site config ────────────────────────────────────────────────────────
sudo tee /etc/nginx/sites-available/videg > /dev/null << NGINXCONF
server {
    listen 80;
    server_name ${DOMAIN};
    # Redirect all HTTP to HTTPS
    return 301 https://\$host\$request_uri;
}

server {
    listen 443 ssl;
    server_name ${DOMAIN};

    # TLS certs (filled in by certbot)
    ssl_certificate     /etc/letsencrypt/live/${DOMAIN}/fullchain.pem;
    ssl_certificate_key /etc/letsencrypt/live/${DOMAIN}/privkey.pem;
    include             /etc/letsencrypt/options-ssl-nginx.conf;
    ssl_dhparam         /etc/letsencrypt/ssl-dhparams.pem;

    # Serve console.html as the root page
    root /home/ubuntu/videg-web;
    index console.html;
    location / {
        try_files \$uri \$uri/ =404;
    }

    # Browser WebSocket  →  relay :8765
    location /ws-browser {
        proxy_pass         http://127.0.0.1:8765;
        proxy_http_version 1.1;
        proxy_set_header   Upgrade    \$http_upgrade;
        proxy_set_header   Connection "upgrade";
        proxy_set_header   Host       \$host;
        proxy_read_timeout 86400s;
    }

    # Edge WebSocket  →  relay :8766
    location /ws-edge {
        proxy_pass         http://127.0.0.1:8766;
        proxy_http_version 1.1;
        proxy_set_header   Upgrade    \$http_upgrade;
        proxy_set_header   Connection "upgrade";
        proxy_set_header   Host       \$host;
        proxy_read_timeout 86400s;
    }
}
NGINXCONF

sudo ln -sf /etc/nginx/sites-available/videg /etc/nginx/sites-enabled/videg
sudo rm -f /etc/nginx/sites-enabled/default

# ── Test nginx config ────────────────────────────────────────────────────────
sudo nginx -t

# ── Get Let's Encrypt cert ───────────────────────────────────────────────────
# This uses the HTTP-01 challenge (nginx must be running, port 80 must be open)
sudo systemctl start nginx
sudo certbot --nginx -d "$DOMAIN" --non-interactive --agree-tos -m "$EMAIL"

# ── Reload nginx with SSL ─────────────────────────────────────────────────────
sudo nginx -s reload

echo ""
echo "════════════════════════════════════════════════════════"
echo "  Done! Next steps:"
echo ""
echo "  1. From your LOCAL machine, upload console.html:"
echo "     scp -i ~/projects/ViDeG/ViDeG.pem \\"
echo "         ~/projects/ViDeG/web/console.html \\"
echo "         ubuntu@${DOMAIN}:~/videg-web/"
echo ""
echo "  2. Restart relay on localhost only:"
echo "     screen -r relay   (Ctrl-C to stop old one)"
echo "     python3 ~/videg_relay.py --host 127.0.0.1"
echo "     Ctrl-A, D to detach"
echo ""
echo "  3. Update start_edge.sh on your local machine:"
echo "     --cloud wss://${DOMAIN}/ws-edge"
echo ""
echo "  4. Open in browser (from anywhere!):"
echo "     https://${DOMAIN}/console.html"
echo "     WS URL: wss://${DOMAIN}/ws-browser"
echo "════════════════════════════════════════════════════════"
