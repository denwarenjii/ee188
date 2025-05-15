-------------------------------------------------------------------------------
-- ANSI Escape Sequence Constants                                             -
--                                                                            -
--  Revision History:                                                         -
--    Date:         Author:    Description:                                   -
--    19 Feb 2025   Chris M.   Initial revision                               -
-------------------------------------------------------------------------------

library std;

use std.standard.all;

package ANSIEscape is
  
  constant ANSI_RESET     : string :=  ESC & '[' & '0' & 'm';        -- ESC[0m
  constant BLACK          : string :=  ESC & '[' & '3' & '0' & 'm';  -- ESC[30m
  constant RED            : string :=  ESC & '[' & '3' & '1' & 'm';  -- ESC[31m
  constant GREEN          : string :=  ESC & '[' & '3' & '2' & 'm';  -- ESC[32m
  constant YELLOW         : string :=  ESC & '[' & '3' & '3' & 'm';  -- ESC[33m
  constant BLUE           : string :=  ESC & '[' & '3' & '4' & 'm';  -- ESC[34m
  constant MAGENTA        : string :=  ESC & '[' & '3' & '5' & 'm';  -- ESC[35m
  constant CYAN           : string :=  ESC & '[' & '3' & '6' & 'm';  -- ESC[36m
  constant WHITE          : string :=  ESC & '[' & '3' & '7' & 'm';  -- ESC[37m
  constant DEFAULT_COLOR  : string :=  ESC & '[' & '3' & '7' & 'm';  -- ESC[37m

end ANSIEscape;

