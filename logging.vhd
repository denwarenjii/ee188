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

  -- Log file;
  file LogFile : text open write_mode is "log.txt";

  -- Log to stdout
  procedure Log(message : in string; 
                Enable  : boolean := true);

  procedure Log(l       : inout line; 
                message : in string; 
                Enable : boolean := true);

  -- Log to a file.
  procedure Log(message          : in string; 
                file file_handle : text; 
                Enable           : boolean := true);

  procedure Log(l                : inout line; 
                message          : in string; 
                file file_handle : text; Enable : boolean := true);

  -- Log to stdout and prefix the message with the current time.
  procedure LogWithTime(message : in string; 
                        Enable  : boolean := true);

  procedure LogWithTime(l       : inout line; 
                        message : in string;
                        Enable  : boolean := true);

  -- Log to a file and prefix the message with the current time.
  procedure LogWithTime(message          : in string; 
                        file file_handle : text; 
                        Enable           : boolean := true);

  procedure LogWithTime(l                : inout line; 
                        message          : in string; 
                        file file_handle : text; 
                        Enable           : boolean := true);

  -- Log to both stdout and a file.
  procedure LogBoth(message          : in string; 
                    file file_handle : text;
                    Enable           : boolean := true);

  procedure LogBoth(l_1              : inout line; 
                    l_2              : inout line; 
                    message          : in string; 
                    file file_handle : text;
                    Enable           : boolean := true);

  -- Log to both stdout and a file and prefix the message with the current time.
  procedure LogBothWithTime(message          : in string; 
                            file file_handle : text; 
                            Enable           : boolean := true);

  procedure LogBothWithTime(l_1 : inout line; 
                            l_2 : inout line; 
                            message : in string; 
                            file file_handle : text;
                            Enable : boolean := true);

end package Logging;


library std;
use std.textio.all;

package body Logging is

  -- Log to stdout
  procedure Log(message : in string; 
                Enable  : boolean := true) is
    variable l : line;
  begin
    if (not Enable) then
      null;
    else
      write(l, message);
      writeline(output, l);
    end if;
  end procedure Log;

  procedure Log(l       : inout line; 
                message : in string; 
                Enable : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
      write(l, message);
      writeline(output, l);
    end if;
  end procedure Log;

  -- Log to a file.
  procedure Log(message          : in string; 
                file file_handle : text; 
                Enable           : boolean := true) is
    variable l : line;
  begin
    if (not Enable) then
      null;
    else
      write(l, message);
      writeline(file_handle, l);
    end if;
  end procedure Log;

  procedure Log(l                : inout line; 
                message          : in string; 
                file file_handle : text; Enable : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
      write(l, message);
      writeline(file_handle, l);
    end if;
  end procedure Log;

  -- Log to stdout and prefix the message with the current time.
  procedure LogWithTime(message : in string; 
                        Enable  : boolean := true) is
    variable l : line;
  begin
    if (not Enable) then
      null;
    else
      write(l, string'("[@"));
      write(l, now);
      write(l, string'("] "));
      write(l, message);
      writeline(output, l);
    end if;
  end procedure LogWithTime;

  procedure LogWithTime(l       : inout line; 
                        message : in string; 
                        Enable  : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
      write(l, string'("[@"));
      write(l, now);
      write(l, string'("] "));
      write(l, message);
      writeline(output, l);
    end if;
  end procedure LogWithTime;

  -- Log to a file and prefix the message with the current time.
  procedure LogWithTime(message          : in string; 
                        file file_handle : text; 
                        Enable           : boolean := true) is
    variable l : line;
  begin
    if (not Enable) then
      null;
    else
      write(l, string'("[@"));
      write(l, now);
      write(l, string'("] "));
      write(l, message);
      writeline(file_handle, l);
    end if;
  end procedure LogWithTime;

  procedure LogWithTime(l                : inout line; 
                        message          : in string; 
                        file file_handle : text; 
                        Enable           : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
      write(l, string'("[@"));
      write(l, now);
      write(l, string'("] "));
      write(l, message);
      writeline(file_handle, l);
    end if;
  end procedure LogWithTime;

  -- Log to both stdout and a file.
  procedure LogBoth(message          : in string; 
                    file file_handle : text;
                    Enable           : boolean := true) is
    variable l_1 : line;
    variable l_2 : line;
  begin
    if (not Enable) then
      null;
    else
      write(l_1, message);
      writeline(file_handle, l_1);
      writeline(output, l_2);
    end if;
  end procedure LogBoth;

  procedure LogBoth(l_1              : inout line; 
                    l_2              : inout line; 
                    message          : in string; 
                    file file_handle : text;
                    Enable           : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
      write(l_1, message);
      writeline(file_handle, l_1);
      writeline(output, l_2);
    end if;
  end procedure LogBoth;

  -- Log to both stdout and a file and prefix the message with the current time.
  procedure LogBothWithTime(message          : in string; 
                            file file_handle : text; 
                            Enable           : boolean := true) is
    variable l_1 : line;
    variable l_2 : line;
  begin
    if (not Enable) then
      null;
    else
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
    end if;
  end procedure LogBothWithTime;

  procedure LogBothWithTime(l_1 : inout line; 
                            l_2 : inout line; 
                            message : in string; 
                            file file_handle : text;
                            Enable : boolean := true) is
  begin
    if (not Enable) then
      null;
    else
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
    end if;
  end procedure LogBothWithTime;

end package body Logging;
