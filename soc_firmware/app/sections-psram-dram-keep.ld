/*
 * Kernel objects that must stay in internal DRAM although their library's
 * .bss is relocated to PSRAM via CONFIG_ESP_SPIRAM_BSS_RELOC_LIBS_AND_OBJS.
 *
 * Linked at the RAM_SECTIONS slot of the esp32s3 linker script, which sits
 * BEFORE .dram0.bss (and therefore before the EXCLUDE_FILE rule that pushes
 * the listed libraries to .ext_ram.data): first match wins, so the input
 * sections named here land in DRAM, the rest of the library falls through
 * to PSRAM.
 */
#ifdef CONFIG_ESP_SPIRAM
  .dram0.psram_keep_bss (NOLOAD) :
  {
    . = ALIGN(4);
    /* HTTP server thread control block (its stack is .noinit and never
     * leaves DRAM; only .bss is relocated). */
    *libsubsys__net__lib__http.a:http_server_core.c.obj(.bss._k_thread_obj_http_server_tid)
    . = ALIGN(4);
  } GROUP_LINK_IN(RAMABLE_REGION)
#endif /* CONFIG_ESP_SPIRAM */
