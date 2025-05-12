----------------------------------------------------------------------------
-- logging.vhd
--
-- Logging Utility package.
--
--  Revision History:
--    12 May 25  Chris M. Initial reivision.
--
----------------------------------------------------------------------------
library std;
use std.textio.all;

package Logging is

  -- Log to stdout
  procedure Log(message : in string);

  -- Log to a file.
  procedure Log(message : in string; file file_handle : text);

  -- Log to stdout and prefix the message with the current time.
  procedure LogWithTime(message : in string);

  -- Log to a file and prefix the message with the current time.
  procedure LogWithTime(message : in string; file file_handle : text);

  -- Log to both stdout and a file.
  procedure LogBoth(message : in string; file file_handle : text);

  -- Log to both stdout and a file and prefix the message with the current time.
  procedure LogBothWithTime(message : in string; file file_handle : text);

end package Logging;


library std;
use std.textio.all;

package body Logging is

  -- Log to stdout
  procedure Log(message : in string) is
    variable l : line;
  begin
    write(l, message);
    writeline(output, l);
  end procedure Log;

  -- Log to a file.
  procedure Log(message : in string; file file_handle : text) is
    variable l : line;
  begin
    write(l, message);
    writeline(file_handle, l);
  end procedure Log;

  -- Log to stdout and prefix the message with the current time.
  procedure LogWithTime(message : in string) is
    variable l : line;
  begin
    write(l, string'("[@"));
    write(l, now);
    write(l, string'("] "));
    write(l, message);
    writeline(output, l);
  end procedure LogWithTime;

  -- Log to a file and prefix the message with the current time.
  procedure LogWithTime(message : in string; file file_handle : text) is
    variable l : line;
  begin
    write(l, string'("[@"));
    write(l, now);
    write(l, string'("] "));
    write(l, message);
    writeline(file_handle, l);
  end procedure LogWithTime;

  -- Log to both stdout and a file.
  procedure LogBoth(message : in string; file file_handle : text) is
    variable l_1 : line;
    variable l_2 : line;
  begin
    write(l_1, message);
    writeline(file_handle, l_1);
    writeline(output, l_2);
  end procedure LogBoth;

  -- Log to both stdout and a file and prefix the message with the current time.
  procedure LogBothWithTime(message : in string; file file_handle : text) is
    variable l_1 : line;
    variable l_2 : line;
  begin
    write(l_1, string'("[@"));
    write(l_1, now);
    write(l_1, string'("] "));
    write(l_1, message);

    write(l_2, string'("[@"));
    write(l_2, now);
    write(l_2, string'("] "));
    write(l_2, message);

    writeline(output, l_1);
    writeline(file_handle, l_2);

  end procedure LogBothWithTime;

end package body Logging;
