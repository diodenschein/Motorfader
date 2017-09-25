/* 8Mhz RC fuse definitions
 * include after 
 */

FUSES =
{
  /* default fuse setting is internal 8Mhz RC div 8 */
  .low = (LFUSE_DEFAULT | ~FUSE_CKDIV8),
  .high = HFUSE_DEFAULT,
  .extended = EFUSE_DEFAULT
};

#define F_CPU 8000000UL