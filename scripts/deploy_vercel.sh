#!/bin/sh
# Build the static arena and ship it to production.
#   https://roborun-arena.vercel.app  (project domain, auto-assigned)
# One-time setup already done: site/.vercel links ps-projects-0c7bba7e/roborun.
set -e
cd "$(dirname "$0")/.."
python3 scripts/build_site.py
cd site
npx -y vercel@latest deploy --prod --yes
echo "live: https://roborun-arena.vercel.app"
