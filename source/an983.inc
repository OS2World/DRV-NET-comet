; *** ADMtek AN983 centaur definition ***

AN983_register	struc
PAR		dd	?	; 00 PCI Access register
		dd	?
TDR		dd	?	; 08 Transmit Demand register
		dd	?
RDR		dd	?	; 10 Receive Demand register
		dd	?
RDB		dd	?	; 18 Receive Descriptor Base Address
		dd	?
TDB		dd	?	; 20 Transmit Descriptor Base Address
		dd	?
SR		dd	?	; 28 Status register
		dd	?
NAR		dd	?	; 30 Network Access register
		dd	?
IER		dd	?	; 38 Interrupt Enable register
		dd	?
LPC		dd	?	; 40 Lost Packet Counter
		dd	?
SPR		dd	?	; 48 Serial Port register
		dd	?,?,?
TMR		dd	?	; 58 Timer
		dd	?,?,?
WCSR		dd	?	; 68 Wake-up Control/Status register
		dd	?
WPDR		dd	?	; 70 Wake-up Pattern Data register
		dd	?
WTMR		dd	?	; 78 Watchdog Timer
		dd	?
ASR2		dd	?	; 80 Status register 2
AIER2		dd	?	; 84 Interrupt Enble register 2
CR		dd	?	; 88 Command register
PCIC		dd	?	; 8c PCI Bus Performance counter
PMCSR		dd	?	; 90 Power Management Command and Status
WTDP		dd	?	; 94 Current Transmit Descriptor Pointer
WRDP		dd	?	; 98 Current Receive Descriptor Pointer
TXBR		dd	?	; 9c Transmit Burst Counter/Time-out register
FROM		dd	?	; a0 Flash ROM port
PAR0		dd	?	; a4 Physical Address register 0
PAR1		dd	?	; a8 Physical Address register 1
MAR0		dd	?	; ac Multicast Address Hash Table register 0
MAR1		dd	?	; b0 Multicast Address Hash Table register 1
UAR0		dd	?	; b4 Unicast Address Hash Table register 0
UAR1		dd	?	; b8 Unicast Address Hash Table register 1
		dd	16 dup(?)
OPR		dd	?	; fc Operation Mode register
AN983_register	ends

; --- PCI Access register ---
MWIE		equ	1000000h	; Memory Write and Invalidate Enable
MRLE		equ	 800000h	; Memory Read Line Enable
MRME		equ	 200000h	; Memory Read Multiple Enable
TAP		equ	  60000h	; Transmit auto-polling (mask)
CAL		equ	  0c000h	; Cache align (mask)
PBL		equ	   3f00h	; Programmable Burst Length (mask)
BLE		equ	     80h	; Big or Litte Endian
DSL		equ	     7ch	; Descriptor Skip Length (mask)
BAR		equ	      2		; Bus Arbitration  0:rx high 1:tx high
SWR		equ	      1		; Software Reset

; --- Status/Interrupt Enable register ---
; +++ Status only +++
BET		equ	3800000h	; Bus Error Type (mask)
TS		equ	 700000h	; Transmit Status (mask)
RS		equ	 0e0000h	; Receive Status (mask)
; +++ Status/Interrupt Enable +++
NISS		equ	  10000h	; Normal Interrupt Status Summary
AISS		equ	   8000h	; Abnormal Interrupt Status Summary
FBE		equ	   2000h	; Fatal Bus Error
GPTT		equ	    800h	; General Purpose Timer Time-out
RWT		equ	    200h	; Receive Watchdog Time-out
RPS		equ	    100h	; Receive Process Stopped
RDU		equ	     80h	; Receive Descriptor Unavailable
RCI		equ	     40h	; Receive Complete Interrupt
TUF		equ	     20h	; Transmit Under-flow
TJT		equ	      8		; Transmit Jabber Timer Time-out
TDU		equ	      4		; Transmit Descriptor Unavailable
TPS		equ	      2		; Transmit Process Stopped
TCI		equ	      1		; Transmit Complete Interrupt

TS_suspend	equ	110b shl 20	; Transmit suspended
RS_suspend	equ	100b shl 17	; Receive suspended

; --- (Assistant) Status/Interrupt Enable register 2 ---
TEI		equ	80000000h	; Transmit Early Interrupt
REI		equ	40000000h	; Receive Early Interrupt
LCS		equ	20000000h	; Status of Link Status Change
TDIS		equ	10000000h	; Transmit Deferred Interrupt status
PF		equ	 4000000h	; Pause Frame Received Interrupt
ANISS		equ	   10000h	; Added Normal Interrupt summary
AAISS		equ	    8000h	; Added Abnormal Interrupt summary

; --- Network access register ---
SF		equ	200000h	; Store and Forward
SQE		equ	 80000h	; SQE disable
TR		equ	 0c000h	; Transmit Threshold control (mask)
STx		equ	  2000h	; Stop Transmit  0:stop 1:start
FC		equ	  1000h	; Force Collision Mode
OM		equ	  0c00h	; Operation Mode (mask)  00:normal 01:loopback
MM		equ	    80h	; Multicast Mode (receive all multicast)
PR		equ	    40h	; Promiscous Mode
SBC		equ	    20h	; Stop Back-off Counter
PB		equ	     8	; Pass Bad Packet
SRx		equ	     2	; Stop Receive  0:stop 1:start

; --- Lost Packet Counter ---
LPCO		equ	10000h	; Lost Packet Counter Overflow
LPC_mask	equ	0ffffh	; Lost Packet Counter

; --- Serial Port register ---
MDI		equ	80000h	; MII Management Data Input
MMC		equ	40000h	; MII Management Control  0:write 1:read
MDO		equ	20000h	; MII Management Data Output
MDC		equ	10000h	; MII Management Clock
SRC		equ	 4000h	; Serial EEPROM Read Control
SWC		equ	 2000h	; Serial EEPROM Write Control
SRS		equ	  800h	; Serial EEPROM Select
SDO		equ	    8	; Serial EEPROM Data Out
SDI		equ	    4	; Serial EEPROM Data In
SCLK		equ	    2	; Serial EEPROM Clock
SCS		equ	    1	; Serial EEPROM Chip Select

; --- General Purpose Timer ---
COM		equ	10000h	; Continuous Operation Mode
GTV		equ	0ffffh	; General Purpose Timer Value (mask)

; --- Wake-up Control/Status register ---
CRCT		equ	40000000h	; CRC16 Type
WP1E		equ	20000000h	; Wake-up Pattern One Matched Enable
WP2E		equ	10000000h	; Wake-up Pattern Two Matched Enable
WP3E		equ	 8000000h	; Wake-up Pattern Three Matched Enable
WP4E		equ	 4000000h	; Wake-up Pattern Four Matched Enable
WP5E		equ	 2000000h	; Wake-up Pattern Five Matched Enable
LinkOFF		equ	   20000h	; Link Off Detect Enable
LinkON		equ	   10000h	; Link On Detect Enable
WFRE		equ	     400h	; Wake-up Frame Received Enable
MPRE		equ	     200h	; Magic Packet Received Enable
LSCE		equ	     100h	; Link Status Changed Enable
WFR		equ	       4	; Wake-up Frame Received
MPR		equ	       2	; Magic Packet Received
LSC		equ	       1	; Link Status Changed

; --- Watchdog Timer ---
RWR		equ	20h	; Receive Watchdog Release
RWD		equ	10h	; Receive Watchdog Disable
JCLK		equ	 4	; Jabber Clock
NJ		equ	 2	; Non-Jabber
JBD		equ	 1	; Jabber Disable

; --- Command register ---
D3CS		equ	80000000h
AUXCL		equ	70000000h	; Aux Current (mask)
Pci_pad		equ	 2000000h
Pmes_sticky	equ	 1000000h
LED43		equ	  800000h	; LED mode
RFS		equ	  600000h	; Receive FIFO size control (mask)
CRD		equ	  100000h	; Clock Run Disable
PME		equ	   80000h	; Power Management enable
APM		equ	   40000h	; APM mode  Magic Packet
LWS		equ	   20000h	; should be 0
D3_APM		equ	      80h
RWP		equ	      40h	; Reset Wake-up Pattern Data Pointer
PAUSE		equ	      20h	; Pause function control
RTE		equ	      10h	; Receive Threshold Enable
DRT		equ	      0ch	; Drain Receive Threshold
SINT		equ	       2	; Software Interrupt
ATUR		equ	       1	; Auto Transmit-Underrun Recovery

; --- PCI Bus Performance counter ---
CLKCNT		equ	0ffff0000h	; PCI clock from last read(read clear)
DWCNT		equ	      0ffh	; Dword by Bus Master from last read

; --- Power Management Command and Status (same as PCI register) ---
PMES		equ	8000h	; PME_STS
DSCAL		equ	6000h	; Data_Scale (mask)
DSEL		equ	1e00h	; Data_Select (mask)
PME_E		equ	 100h	; PME_EN
PWRS		equ	   3	; Power State (mask)

; --- Transmit Burst Count/ Time-out ---
TBCNT		equ	1f0000h	; Transmit Burst Count (mask)
TTO		equ	  0fffh	; Transmit Time-out (mask)

; --- Flash ROM port ---
Bra16_on	equ	80000000h
REN		equ	 8000000h	; Read Enable
WEN		equ	 4000000h	; Write Enable
flADD		equ	 3ffff00h	; Flash ROM address (mask)
flDATA		equ	     0ffh	; Read/Write Data (mask)

; --- Operation Mode register ---
SPEED		equ	80000000h	; Network Speed Status  1:100 0:10
FD		equ	40000000h	; Network Duplex Status  1:full 0:half
LINK		equ	20000000h	; Network Link Status  1:on 0:off
EERLOD		equ	 4000000h	; EEPROM data reload
OpMode		equ	       7	; Operation Mode  111:single 100:MAC


; --- receive descriptor ---
rxd		struc
sts		dd	?	; status
ctl		dd	?	; control
buf		dd	?	; buffer1 address
link		dd	?	; next link (buffer 2 address)
; above for device, below for driver
physadr		dd	?	; physical address of this descriptor
vbuf		dd	?	; virtual address of buffer1
tail		dw	?	; frame tail descriptor
vlink		dw	?	; virtual link
rxd		ends

; --- transmit descriptor ---
txd		struc
sts		dd	?	; status
ctl		dd	?	; control
buf		dd	?	; buffer1 address
link		dd	?	; next link (buffer2 address)
; above for device, below for driver
physadr		dd	?	; physical address of this descriptor
vlink		dw	?	; virtual link
		dw	?	; reserved
txd		ends

; --- virtual transmit descriptor (driver purpose) ---
vtxd		struc
head		dw	?	; head descriptor of a frame
tail		dw	?	; tail descriptor of a frame
reqhandle	dw	?	; request handle
protid		dw	?	; protocol id
cnt		dw	?	; descriptor count
vlink		dw	?	; virtual link
immedphys	dd	?	; physical address of immediate data buffer
immed		db	64 dup(?) ; reserved buffer for immediate data
vtxd		ends

; --- tx/rx status/control definition ---
; +++ status +++
OWN		equ	80000000h
ErrSum		equ	    8000h	; Error summary
; +++ control +++
EndRing		equ	2000000h	; end of ring
CHain		equ	1000000h	; 2nd address chain
BS2		equ	 3ff800h	; buffer2 size (10bit mask)
BS1		equ	    7ffh	; buffer1 size (10bit mask)

; --- rx status/control definition ---
; +++ status +++
rxFL		equ	7fff0000h	; frame length (including CRC) (mask)
rxDE		equ	    4000h	; Descriptor Error
rxDT		equ	    3000h	; Data Type  00:normal (mask)
rxRF		equ	     800h	; Runt Frame
rxMF		equ	     400h	; Multicast Frame
rxFS		equ	     200h	; First Descriptor
rxLS		equ	     100h	; Last Descriptor
rxTL		equ	      80h	; Too Long frame
rxCS		equ	      40h	; Late Collision
rxFT		equ	      20h	; Frame Type  1:ethernet 0:802.3
rxRW		equ	      10h	; Receive Watchdog
rxDB		equ	       4	; Dribble bit
rxCE		equ	       2	; CRC Error
rxOF		equ	       1	; Overflow

; --- tx status/control definition ---
; +++ status +++
txUR		equ	0c00000h	; Under-run count (mask)
txTO		equ	   4000h	; Jabber time-out
txLO		equ	    800h	; Loss carrier
txNC		equ	    400h	; No carrier
txLC		equ	    200h	; Late Collision
txEC		equ	    100h	; Excessive collision
txHF		equ	     80h	; Heartbeat fail
txCC		equ	     78h	; Collision Count (mask)
txUF		equ	      2		; Under-run error
txDE		equ	      1		; Deferred
; +++ control +++
txIC		equ	80000000h	; Interrupt Completed
txLS		equ	40000000h	; Last descriptor
txFS		equ	20000000h	; First descriptor
txAC		equ	 4000000h	; Disable add CRC function
txDPD		equ	  800000h	; Disable Padding function



