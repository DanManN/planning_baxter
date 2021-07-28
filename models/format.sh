#!/usr/bin/env bash
for x in $(ls */*.sdf)
do
	xmllint --format $x -o $x
done
