# Contact Sensor

## About
This Z-Wave 700-series based contact sensor "Reference Design" is based on a custom built hardware that is provided along with the software based on SDK 7.14.3.0 (Simplicity Studio v5) in this repository.<br>
Two hardware variations are provided:<br>
<ul>
<li> Reed Switch (SPST) option</li>
<li> Silabs Hall Sensor (Si7210-B-00-IV) option</li>
</ul>
Other hardware components:
<ul>
<li>1 temperature & humidity sensor (Si7021 OR model tbd.)</li>
<li>1 programming button (functionality for: wakeup / inclusion / exclusion / factory reset)</li>
<li>1 tampering button (detect removal of housing)</li>
<li>1 Status LED</li>
<li>1 Chip Antenna (customer version)</li>
<li>1 PCB Antenna (Silabs version)</li>
</ul>

```
#elif defined(CUSTOM_BOARD)
#include "custom_board.h"
```

## Project Setup

### Hardware Prerequisites for Debugging
<ul>
  <li>WSTK Development Board</li>
  <li>Custom board</li>
  <li>Simplicity Debug Adapter Board</li>
</ul>

### Software Prerequisites
<ul>
  <li>Simplicity Studio v5 (SSv5)</li>
  <li>Gecko SDK Suite: Z-Wave SDK 7.14.3.0</li>
</ul>

### Required Modifications
2 files that are part of the SDK need to be modified after importing the project into SSv5<br>
board.h: At line 28 & 29 add:<br>

```
#elif defined(CUSTOM_BOARD)
#include "custom_board.h"
```

board.c: At line 595: remove the "static" keyword from
```
static bool ButtonEnableEM4PinWakeup()
```
The project should now build without any errors or warnings.

# Reporting Bugs/Issues and Posting Questions and Comments
<ul>
  To report bugs in the Application Examples projects, please create a new "Issue" in the "Issues" section of this repo. Please reference the board, project, and source files associated with the bug, and reference line numbers. If you are proposing a fix, also include information on the proposed fix. Since these examples are provided as-is, there is no guarantee that these examples will be updated to fix these issues.
</ul>
<ul>
  Questions and comments related to these examples should be made by creating a new "Issue" in the "Issues" section of this repo.´
</ul>

# Disclaimer
<ul>
  The Gecko SDK suite supports development with Silicon Labs IoT SoC and module devices. Unless otherwise specified in the specific directory, all examples are considered to be EXPERIMENTAL QUALITY which implies that the code provided in the repos has not been formally tested and is provided as-is. It is not suitable for production environments. In addition, this code will not be maintained and there may be no bug maintenance planned for these resources. Silicon Labs may update projects from time to time.
</ul>