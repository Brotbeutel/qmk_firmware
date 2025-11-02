#!/bin/bash
git add .
git commit -m "${1:-Auto-Commit}"
git push
chmod +x push.sh
