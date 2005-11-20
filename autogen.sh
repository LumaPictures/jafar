#! /bin/sh

set -e
echo "*** Running aclocal"
aclocal -I aclocal
echo "*** Running autoconf"
autoconf

