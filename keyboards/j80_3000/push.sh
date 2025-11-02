#!/bin/bash

# This is a script to automate the git commit function. When you're in this directory: C:/Users/Student/qmk_firmware/keyboards/j80_3000,
# you can run this command: ./push.sh "Your message of changes here. (optional)"

git add .
git commit -m "${1:-Auto-Commit}"
git push
