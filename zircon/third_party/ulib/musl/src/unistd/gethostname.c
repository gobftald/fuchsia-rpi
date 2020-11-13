#include <sys/utsname.h>
#include <unistd.h>
#include <stdio.h>

int gethostname(char* name, size_t len) {
  size_t i;
  struct utsname uts;
  printf("# gethostname: uname(&uts)\n");
  if (uname(&uts))
    return -1;
  printf("# gethostname: for (i = 0; i < len && (name[i] = uts.nodename[i]); i++)\n");
  if (len > sizeof uts.nodename)
    len = sizeof uts.nodename;
  for (i = 0; i < len && (name[i] = uts.nodename[i]); i++)
    ;
  if (i == len)
    name[i - 1] = 0;
  printf("# gethostname: name = %s\n", name);
  return 0;
}
