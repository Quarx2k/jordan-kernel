#
# Makefile for the LTT objects.
#

obj-$(CONFIG_MARKERS)			+= ltt-channels.o
obj-$(CONFIG_LTT)			+= ltt-core.o
obj-$(CONFIG_LTT_TRACER)		+= ltt-tracer.o
obj-$(CONFIG_LTT_TRACE_CONTROL)		+= ltt-marker-control.o

ifdef CONFIG_LTT_RELAY_LOCKLESS
RELAY_LOCKING := ltt-relay-lockless.o
endif

ifdef CONFIG_LTT_RELAY_IRQOFF
RELAY_LOCKING := ltt-relay-irqoff.o
endif

ifdef CONFIG_LTT_RELAY_LOCKED
RELAY_LOCKING := ltt-relay-locked.o
endif

obj-$(CONFIG_LTT_RELAY) += ltt-relay.o
ltt-relay-objs := $(RELAY_LOCKING) ltt-relay-alloc.o ltt-relay-splice.o \
		  ltt-relay-vfs.o

obj-$(CONFIG_LTT_SERIALIZE)		+= ltt-serialize.o
obj-$(CONFIG_LTT_STATEDUMP)		+= ltt-statedump.o
obj-$(CONFIG_LTT_FAST_SERIALIZE)	+= ltt-type-serializer.o
obj-$(CONFIG_LTT_TRACE_CONTROL)		+= ltt-trace-control.o
obj-$(CONFIG_LTT_USERSPACE_EVENT)	+= ltt-userspace-event.o
obj-$(CONFIG_LTT_FILTER)		+= ltt-filter.o
obj-$(CONFIG_LTT_KPROBES)		+= ltt-kprobes.o
obj-$(CONFIG_LTT_TRACEPROBES)		+= probes/
obj-$(CONFIG_LTT_FTRACE)		+= ltt-ftrace.o
obj-$(CONFIG_LTT_ASCII)			+= ltt-ascii.o
