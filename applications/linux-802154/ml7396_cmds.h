#ifndef __ML7396_CMDS_H
#define __ML7396_CMDS_H

int cmd_ml7396_reads(int argc, char **argv);
int cmd_ml7396_get_interrupt_status(int argc, char **argv);
int cmd_ml7396_reset(int argc, char **argv);
int cmd_ml7396_regwrite(int argc, char **argv);
int cmd_ml7396_regread(int argc, char **argv);
int cmd_cca(int argc, char **argv);
int cmd_fifo_write(int argc, char **argv);
int cmd_ml7396_status(int argc, char **argv);

#endif
