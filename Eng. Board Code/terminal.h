#ifndef __TERMINAL_H
#define __TERMINAL_H

  typedef struct{
    const char* name;
    const char* helpStr;
    //function pointer to command
    int (*cmd)(char **argv,unsigned short argc);
  }CMD_SPEC;
 
  //table of commands with help
  extern CMD_SPEC cmd_tbl[];

  int __putchar(int ch);
  unsigned short make_args(char *argv[],const char *src,char *dst);
  int helpCmd(char **argv,unsigned short argc);
  int doCmd(char *cs);

#endif
