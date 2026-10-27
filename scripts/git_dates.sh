#!/bin/bash
cd ~/Desktop/sp_vision_25_hero
echo '=== all refs with full timestamps ==='
git for-each-ref --sort=-committerdate --format='%(refname:short) | %(objectname:short) | %(committerdate:iso8601) | %(contents:subject)' refs/heads refs/remotes
echo
echo '=== master history 09-21/09-22 boundary, full iso dates ==='
git log master --pretty=format:'%h | %ci | %s' -6
echo
echo '=== branches backup_2159 / backup_2209 ==='
git log backup_2159 --pretty=format:'%h | %ci | %s' -3
echo
git log backup_2209 --pretty=format:'%h | %ci | %s' -3