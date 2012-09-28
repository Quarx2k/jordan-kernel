/*
 *  include/linux/socinfo.h
 *
 *  Copyright (C) 2010 Nokia Corporation
 *
 *  Contact: Eduardo Valentin <eduardo.valen...@nokia.com>
 *
 *  proc socinfo file
 */

#ifndef __SOCINFO_H
#define __SOCINFO_H

#include <linux/seq_file.h>

void register_socinfo_show(int (*show)(struct seq_file *, void *), void *data);
#endif
