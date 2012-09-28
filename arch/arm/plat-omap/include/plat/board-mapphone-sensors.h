/*
 * Platform data for mapphone sensors.
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __BOARD_MAPPHONE_SENSORS_H
#define __BOARD_MAPPHONE_SENSORS_H

#include <linux/akm8973.h>
#include <linux/i2c/adp8870.h>
#include <linux/isl29030.h>
#include <linux/kxtf9.h>
#include <linux/lis331dlh.h>

extern struct adp8870_backlight_platform_data adp8870_pdata;
extern struct akm8973_platform_data mapphone_akm8973_data;
extern struct akm8975_platform_data mapphone_akm8975_pdata;
extern struct isl29030_platform_data isl29030_pdata;
extern struct kxtf9_platform_data mapphone_kxtf9_data;
extern struct lis331dlh_platform_data mapphone_lis331dlh_data;

#endif /* __BOARD_MAPPHONE_SENSORS_H */
