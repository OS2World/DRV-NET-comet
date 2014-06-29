; *** ADMtek AL981 comet definition ***

; No MII MDIO bits in SPR.
; PHY registers are mapped into Control/Status Registers.
; No OPR register.

AL981_register	struc
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
		dd	?	; 94 
		dd	?	; 98 
TXBR		dd	?	; 9c Transmit Burst Counter/Time-out register
FROM		dd	?	; a0 Flash ROM port
PAR0		dd	?	; a4 Physical Address register 0
PAR1		dd	?	; a8 Physical Address register 1
MAR0		dd	?	; ac Multicast Address Hash Table register 0
MAR1		dd	?	; b0 Multicast Address Hash Table register 1

XCR		dd	?	; b4  0 Control register
XSR		dd	?	; b8  1 Status register
PID1		dd	?	; bc  2 PHY identifier 1
PID2		dd	?	; c0  3 PHY identifier 2
ANA		dd	?	; c4  4 AN advertisement register
ANLPA		dd	?	; c8  5 AN Link Partner Ability register
ANE		dd	?	; cc  6 AN Expansion register
XMC		dd	?	; d0 10 Mode control register
XCIIS		dd	?	; d4 11 Configuration Info Int Status
XIE		dd	?	; d8 12 Interrupt Enable register
CTR100		dd	?	; dc 13 100BASE-TX PHY control/status
		dd	8 dup(?)
AL981_register	ends

