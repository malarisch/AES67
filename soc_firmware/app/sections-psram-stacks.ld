/*
 * With CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM the esp32s3 linker script routes ALL
 * .noinit input sections of the network libraries into PSRAM (.ext_ram.data),
 * which is what we want for the big net_pkt slabs and net_buf data pools — but
 * .noinit also carries K_KERNEL_STACK_DEFINE thread stacks, and Xtensa cannot
 * safely run threads on external-PSRAM stacks (net_mgmt fataled in k_sem_give
 * with SP = 0x3c06xxxx once its stack landed there).
 *
 * This snippet is linked between .dram0.noinit (which EXCLUDE_FILEs the net
 * libraries) and .ext_ram.data (which catches them), so first-match-wins pulls
 * just the stack-owning object files back to internal DRAM; everything else in
 * those libraries still falls through to PSRAM.
 */
#ifdef CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM
  .dram0.net_stacks_noinit (NOLOAD) :
  {
    . = ALIGN(4);
    /* net_tc.c: RX/TX traffic-class queue stacks; net_mgmt.c: mgmt_stack;
     * net_if.c: tx_ts_stack; tcp.c: work_q_stack (drags the tcp_conns slab
     * along — harmless); sockets_service.c: service thread stack. */
    *libsubsys__net__ip.a:net_tc.c.obj(.noinit .noinit.*)
    *libsubsys__net__ip.a:net_mgmt.c.obj(.noinit .noinit.*)
    *libsubsys__net__ip.a:net_if.c.obj(.noinit .noinit.*)
    *libsubsys__net__ip.a:tcp.c.obj(.noinit .noinit.*)
    *libsubsys__net.a:sockets_service.c.obj(.noinit .noinit.*)
    . = ALIGN(4);
  } GROUP_LINK_IN(RAMABLE_REGION)
#endif /* CONFIG_ESP32_WIFI_NET_ALLOC_SPIRAM */
