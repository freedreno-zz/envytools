#!/bin/bash


for f in $*; do
	d=`dirname $f`
	b=`basename $f`
	num=${b#shader-runner-}
	num=${num%.rd.gz}
	num=${num%.rd}
	num=${num##0}
	#echo "manhattan/$num.shader_test - "
	pgmdump2 --shaderdb $f 2>&1 1> /dev/null | sed "s/.*/$d\/$num.shader_test - &\n/"
done
