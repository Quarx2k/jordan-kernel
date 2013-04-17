#!/usr/bin/python
#
# Copyright (C) 2013 Motorola, LLC.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307, USA
#
# generate a 64 bit aligned concatenation of the files
# given based on:
# padding = (align - (offset mod align)) mod align
import os, sys


def padding(v):
    align = 8
    return (align - (v % align)) % align

for f in sys.argv[1:]:
    s = os.stat(f).st_size
    print >> sys.stderr, f, 'size', s, 'padding', padding(s)
    with open(f, 'rb') as fp:
        data = fp.read()
        sys.stdout.write(data)
        for i in range(padding(s)):
            sys.stdout.write(B'0')

