; *** Resident part: Hardware dependent ***

include	NDISdef.inc
include	an983.inc
include	al981.inc
include	MIIdef.inc
include	misc.inc
include	DrvRes.inc

extern	DosIODelayCnt : far16

public	DrvMajVer, DrvMinVer
DrvMajVer	equ	1
DrvMinVer	equ	3

.386

_REGSTR	segment	use16 dword AT 'RGST'
	org	0
RegCmt	label	AL981_register
Reg	AN983_register <>
_REGSTR	ends

_DATA	segment	public word use16 'DATA'

; --- DMA Descriptor management ---
public	VTxFreeCount, TxFreeCount
public	VTxHead, VTxFreeHead, TxFreeHead
VTxFreeCount	dw	0
TxFreeCount	dw	0
VTxHead		dw	0
VTxFreeHead	dw	0
TxFreeHead	dw	0

public	RxHead, RxTail, RxBusyHead, RxBusyTail, RxInProg
RxInProg	dw	0
RxHead		dw	0
RxTail		dw	0
RxBusyHead	dw	0
RxBusyTail	dw	0

; --- System(PCI) Resource ---
public	IOaddr, MEMSel, MEMaddr, IRQlevel
public	CacheLine, Latency
IOaddr		dw	?
MEMSel		dw	?
MEMaddr		dd	?
IRQlevel	db	?
CacheLine	db	?	; [0..3] <- [0,8,16,32]
Latency		db	?

public	eepType, ChipType	; << for debug >>
eepType		db	?	; serial EEPROM type [0..2]
ChipType	db	?	; 0:comet  1:centaur

align	2
; --- Physical information ---
PhyInfo		_PhyInfo <>

public	MediaSpeed, MediaDuplex, MediaPause, MediaLink	; << for debug >>
MediaSpeed	db	0
MediaDuplex	db	0
MediaPause	db	0
MediaLink	db	0

; --- Register Contents ---
public	regIntStatus, regIntMask	; << for debug info >>
public	regHashTable

regIntStatus	dd	0
regIntMask	dd	0
regHashTable	dw	4 dup (0)

; --- ReceiveChain Frame Descriptor ---
public	RxFrameLen, RxDesc	; << for debug info >>
RxFrameLen	dw	0
RxDesc		RxFrameDesc	<>


; --- Configuration Memory Image Parameters ---
public	cfgSLOT, cfgTXQUEUE, cfgRXQUEUE, cfgMAXFRAMESIZE
public	cfgTxDRTH, cfgRxDRTH, cfgMXDMA, cfgTAP
public	cfgPCIMWI, cfgPCIMRL, cfgPCIMRM, cfgPCIBAP
public	cfgTBCNT, cfgTTO
public	cfgRxAcErr
cfgSLOT		db	0
cfgTXQUEUE	db	8
cfgRXQUEUE	db	16

cfgTxDRTH	db	11b	; 1024bytes [0..4] <- [128,256,512,1024,S&F]
cfgRxDRTH	db	01b	; 64bytes  [0..2] <- [32,64,S&F]

cfgPCIMWI	db	1	; Memory Write and Invalidate
cfgPCIMRL	db	1	; Memory Read cache line
cfgPCIMRM	db	1	; Memory Read Multiple
cfgMXDMA	db	010000b	; 16dwords  [0..20] 
cfgTAP		db	0	; [0..3] <- [disable(0),200,800,1600]
cfgPCIBAP	db	0	; rx high priority

cfgTBCNT	db	3	; [0..1f]
cfgTTO		dw	300	; n*2.56us 768us [0..fff]

cfgRxAcErr	dw	rxDE or rxRF or rxCS or rxDB or rxCE or rxOF
cfgMAXFRAMESIZE	dw	1514

; --- Receive Buffer address ---
public	RxBufferLin, RxBufferPhys, RxBufferSize, RxBufferSelCnt, RxBufferSel
RxBufferLin	dd	?
RxBufferPhys	dd	?
RxBufferSize	dd	?
RxBufferSelCnt	dw	?
RxBufferSel	dw	2 dup (?)	; max is 2.

; ---Vendor Adapter Description ---
public	AdapterDesc
AdapterDesc	db	'ADMTek AN983 Centaur Fast Ethernet Adapter',0


_DATA	ends

_TEXT	segment	public word use16 'CODE'
	assume	ds:_DATA, gs:_REGSTR
	
; USHORT hwTxChain(TxFrameDesc *txd, USHORT rqh, USHORT pid)
_hwTxChain	proc	near
	push	bp
	mov	bp,sp
	push	fs
	lfs	bx,[bp+4]
	xor	ax,ax
	push	offset semTx
	mov	cx,fs:[bx].TxFrameDesc.TxImmedLen
	mov	dx,fs:[bx].TxFrameDesc.TxDataCount
	cmp	ax,cx
	adc	ax,dx		; ax=number of txd required.

	call	_EnterCrit
	mov	si,[VTxFreeCount]
	mov	di,[TxFreeCount]
	dec	si
	jl	short loc_or		; no vtxd, out of resource
	sub	di,ax
	jc	short loc_or		; lack of txd, out of resource
	mov	[VTxFreeCount],si
	mov	[TxFreeCount],di
	mov	bx,[VTxFreeHead]
	mov	si,[TxFreeHead]
	mov	[bx].vtxd.cnt,ax	; fragment count
	mov	[bx].vtxd.head,si	; first fragment
loc_1:
	mov	di,si
	dec	ax
	mov	si,[di].txd.vlink
	jnz	short loc_1
	mov	ax,[bx].vtxd.vlink
	mov	[bx].vtxd.tail,di	; last framgent
	mov	[TxFreeHead],si		; next txd
	mov	[VTxFreeHead],ax	; next vtxd
	mov	di,[bp+8]
	mov	si,[bp+10]
	test	cx,cx			; immediate length
	mov	[bx].vtxd.reqhandle,di
	mov	[bx].vtxd.protid,si
	mov	bp,[bp+4]
	mov	di,[bx].vtxd.head
	jz	short loc_2		; no immediate data

	push	di
	mov	ax,cx
	mov	word ptr [di].txd.ctl,cx
	and	ax,3
	shr	cx,2
	lea	di,[bx].vtxd.immed
	push	ds
	push	ds
	pop	es
	lds	si,fs:[bp].TxFrameDesc.TxImmedPtr
	rep	movsd
	mov	cx,ax
	rep	movsb
	pop	ds
	pop	di
	mov	eax,[bx].vtxd.immedphys
	mov	[di].txd.buf,eax
	jmp	short loc_4

loc_or:
	mov	ax,OUT_OF_RESOURCE
	jmp	short loc_8

loc_2:
	add	bp,offset TxFrameDesc.TxBufDesc1 ; sizeof(TxBufDesc)
	cmp	fs:[bp].TxBufDesc.TxPtrType,0
	mov	eax,fs:[bp].TxBufDesc.TxDataPtr
	jz	short loc_3
	push	eax
	call	_VirtToPhys
	add	sp,4
loc_3:
	mov	dx,fs:[bp].TxBufDesc.TxDataLen
	mov	[di].txd.buf,eax
	mov	word ptr [di].txd.ctl,dx
loc_4:
	mov	ax,highword CHain
	cmp	di,[bx].vtxd.head
	jnz	short loc_5
	or	ax,highword txFS
loc_5:
	cmp	di,[bx].vtxd.tail
	jnz	short loc_6
	or	ax,highword txLS
;	or	ax,highword (txLS or txIC)
loc_6:
	mov	word ptr [di].txd.ctl[2],ax
	mov	word ptr [di].txd.sts[2],highword OWN
	test	ax,highword txLS
	mov	di,[di].txd.vlink
	jz	short loc_2

	mov	ax,word ptr gs:[Reg.SR][2]
	and	ax,highword TS
	cmp	ax,highword TS_suspend	; tx suspended?
	jnz	short loc_7
	mov	gs:[Reg.TDR],eax	; tx polling demand
loc_7:
	mov	ax,REQUEST_QUEUED
loc_8:
	call	_LeaveCrit
	pop	cx	; stack adjust
	pop	fs
	pop	bp
	retn
_hwTxChain	endp


_hwRxRelease	proc	near
	push	bp
	mov	bp,sp
	push	si
	push	di
	push	offset semRx
	call	_EnterCrit

	mov	ax,[bp+4]		; ReqHandle = vrxd
	mov	bx,[RxInProg]
	test	bx,bx
	jz	short loc_1		; no frame in progress
	cmp	ax,bx
	jnz	short loc_1
	mov	di,[bx].rxd.tail
	mov	[RxInProg],0
	jmp	short loc_4

loc_1:
	mov	bx,[RxBusyHead]
loc_2:
	or	bx,bx
	jz	short loc_ex		; not found
	cmp	ax,bx
	jz	short loc_3		; found frame id matched
	mov	si,[bx].rxd.tail
	mov	bx,[si].rxd.vlink
	jmp	short loc_2
loc_3:
	mov	di,[bx].rxd.tail
	cmp	bx,[RxBusyHead]
	mov	ax,[di].rxd.vlink
	jz	short loc_h
	cmp	di,[RxBusyTail]
	jnz	short loc_m
loc_t:
	mov	[RxBusyTail],si
loc_m:
	mov	[si].rxd.vlink,ax
	jmp	short loc_4
loc_h:
	mov	[RxBusyHead],ax
loc_4:
	mov	si,[RxTail]
	mov	eax,[bx].rxd.physadr
	mov	[si].rxd.link,eax
	mov	[si].rxd.vlink,bx
	mov	[RxTail],di
loc_5:
;	mov	word ptr [si].rxd.ctl,1535
;	mov	word ptr [si].rxd.ctl[2],highword CHain
	mov	word ptr [si].rxd.sts[2],highword OWN
	mov	si,[si].rxd.vlink
	cmp	si,di
	jnz	short loc_5
loc_ex:
	mov	ax,word ptr gs:[Reg.SR][2]
	and	ax,highword RS
	cmp	ax,highword RS_suspend
	jnz	short loc_6
	mov	gs:[Reg.RDR],eax
loc_6:
	call	_LeaveCrit
	pop	cx	; stack adjust
	mov	ax,SUCCESS
	pop	di
	pop	si
	pop	bp
	retn
_hwRxRelease	endp


_ServiceIntTx	proc	near
	cld
	push	offset semTx
loc_0:
	call	_EnterCrit
	mov	bx,[VTxHead]
	cmp	bx,[VTxFreeHead]
	jz	short loc_ex		; vtxd queue is empty
	mov	si,[bx].vtxd.tail
	mov	ax,word ptr [si].txd.sts[2]
	test	ax,highword OWN
	jnz	short loc_ex		; incomplete

	mov	cx,[bx].vtxd.cnt
	mov	dx,[bx].vtxd.vlink
	inc	[VTxFreeCount]		; release vtxd
	add	[TxFreeCount],cx	; release txd
	mov	[VTxHead],dx		; update vtxd head
	mov	ax,word ptr [si].txd.sts
	mov	cx,[bx].vtxd.reqhandle
	mov	dx,[bx].vtxd.protid
	call	_LeaveCrit

	test	cx,cx
	jz	short loc_0		; null request handle - no confirm
	shr	ax,15
	mov	bx,[CommonChar.moduleID]
	mov	si,[ProtDS]
	neg	al			; [0,ff] <- ES[0,1]

	push	dx	; ProtID
	push	bx	; MACID
	push	cx	; ReqHandle
	push	ax	; Status
	push	si	; ProtDS
	call	dword ptr [LowDisp.txconfirm]

	mov	gs,[MEMSel]	; fix gs selector
	jmp	short loc_0

loc_ex:
	call	_LeaveCrit
	pop	ax	; stack adjust
	retn
_ServiceIntTx	endp


_ServiceIntRx	proc	near
	push	bp
	push	offset semRx
loc_0:
	call	_EnterCrit
loc_1:
	mov	bx,[RxInProg]
	mov	si,[RxHead]
	or	bx,bx
	jnz	near ptr loc_rty	; retry suspended frame
	cmp	si,[RxTail]
;	jz	short loc_ex		; rx queue unavailable!
	jz	near ptr loc_ex
	mov	ax,word ptr [si].rxd.sts[2]
	test	ax,highword OWN
;	jnz	short loc_ex		; rx queue empty
	jnz	near ptr loc_ex
	mov	ax,word ptr [si].rxd.sts
	test	ax,rxFS
	jz	short loc_rmv		; first descriptor missing - discard
	mov	di,offset RxDesc.RxBufDesc1
	xor	bp,bp
;	mov	[RxDesc.RxDataCount],1
	mov	word ptr [di-2],1
loc_2:
	cmp	[RxDesc.RxDataCount],8
	ja	short loc_rmv		; too many fragment - discard
	mov	cx,word ptr [si].rxd.vbuf
	mov	dx,word ptr [si].rxd.vbuf[2]
	mov	word ptr [di].RxBufDesc.RxDataPtr,cx
	mov	word ptr [di].RxBufDesc.RxDataPtr[2],dx
	test	ax,rxLS
	jnz	short loc_3		; last descriptor found
	mov	cx,word ptr [si].rxd.ctl
	inc	[RxDesc.RxDataCount]
	mov	bx,si
	mov	[di].RxBufDesc.RxDataLen,cx
	mov	si,[si].rxd.vlink
	add	di,sizeof(RxBufDesc)
	add	bp,cx
	cmp	si,[RxTail]
	jz	short loc_ex		; rx queue full - remove? exit!
	mov	ax,word ptr [si].rxd.sts[2]
	test	ax,highword OWN
	jnz	short loc_ex		; processing
	mov	ax,word ptr [si].rxd.sts
	test	ax,rxFS
	jz	short loc_2
				; first before last desc. - discard
	mov	si,bx		; previous pointer
	
loc_rmv:
;	mov	word ptr [si].rxd.sts[2],0	; clear OWN - terminate
	mov	ax,[si].rxd.vlink
	mov	bx,[RxHead]
	mov	di,[RxTail]
	mov	[RxHead],ax
	mov	[RxTail],si
	mov	[di].rxd.vlink,bx		; next link chain
	mov	eax,[bx].rxd.physadr
	mov	[di].rxd.link,eax
loc_rmv1:
;	mov	word ptr [di].rxd.ctl,1536
;	mov	word ptr [di].rxd.ctl[2],highword CHain
	mov	word ptr [di].rxd.sts[2],highword OWN
	mov	di,[di].rxd.vlink
	cmp	si,di
	jnz	short loc_rmv1
	mov	ax,word ptr gs:[Reg.SR][2]
	and	ax,highword RS
	cmp	ax,highword RS_suspend		; if rx is suspended
	jnz	short loc_rmv2
	mov	gs:[Reg.RDR],eax		; poll demand
loc_rmv2:
	jmp	near ptr loc_1

loc_ex:
	call	_LeaveCrit
	pop	cx	; stack adjust
	pop	bp
	retn

loc_3:
	test	ax,[cfgRxAcErr]
	mov	dx,word ptr [si].rxd.sts[2]
	jnz	short loc_rmv		; errored frame - discard
	sub	dx,4			; frame length
	jna	short loc_rmv		; frame length <= 0?
	mov	ax,dx
	cmp	dx,[cfgMAXFRAMESIZE]
	ja	short loc_rmv		; too long length
	sub	dx,bp
	mov	[di].RxBufDesc.RxDataLen,dx
	ja	short loc_4
	dec	[RxDesc.RxDataCount]	; reduce fragment count
	jz	short loc_rmv
	add	[di-sizeof(RxBufDesc)].RxBufDesc.RxDataLen,dx
loc_4:
	mov	bx,[RxHead]
	mov	cx,[si].rxd.vlink
	mov	[RxFrameLen],ax
	mov	[bx].rxd.tail,si
	mov	[RxInProg],bx
	mov	[RxHead],cx
loc_rty:
	call	_LeaveCrit

	call	_IndicationChkOFF
	or	ax,ax
	jz	short loc_spd		; indicate off - suspend...

	push	-1
	mov	bx,[RxInProg]
	mov	cx,[RxFrameLen]
	mov	ax,[ProtDS]
	mov	dx,[CommonChar.moduleID]
	mov	di,sp
	push	bx			; current vrxd = handle

	push	dx		; MACID
	push	cx		; FrameSize
	push	bx		; ReqHandle
	push	ds
	push	offset RxDesc	; RxFrameDesc
	push	ss
	push	di		; Indicate
	push	ax		; Protocol DS
	cld
	call	dword ptr [LowDisp.rxchain]
	mov	gs,[MEMSel]	; fix gs selector
lock	or	[drvflags],mask df_idcp
	cmp	ax,WAIT_FOR_RELEASE
	jz	short loc_6
	call	_hwRxRelease
loc_5:
	pop	cx	; stack adjust
	pop	ax	; indicate
	cmp	al,-1
	jnz	short loc_spd		; indication remains OFF - suspend
	call	_IndicationON
	jmp	near ptr loc_0
loc_6:
	call	_RxPutBusyQueue
	jmp	short loc_5

loc_spd:
lock	or	[drvflags],mask df_rxsp
	pop	cx	; stack adjust
	pop	bp
	retn

_RxPutBusyQueue	proc	near
	push	offset semRx
	call	_EnterCrit
	mov	bx,[RxInProg]
	xor	ax,ax
	test	bx,bx
	jz	short loc_ex		; no progess frame
	mov	di,[bx].rxd.tail
	cmp	ax,[RxBusyHead]
	jnz	short loc_1
	mov	[RxBusyHead],bx
	jmp	short loc_2
loc_1:
	mov	si,[RxBusyTail]
	mov	[si].rxd.vlink,bx
loc_2:
	mov	[RxInProg],ax		; clear
	mov	[RxBusyTail],di
	mov	[di].rxd.vlink,ax	; null pointer
loc_ex:
	call	_LeaveCrit
	pop	bx	; stack adjust
	retn
_RxPutBusyQueue	endp

_ServiceIntRx	endp


_hwServiceInt	proc	near
	enter	2,0
loc_0:
	mov	eax,gs:[Reg.ASR2]
	and	eax,not(ANISS or AAISS)
lock	or	[regIntStatus],eax
	mov	eax,[regIntStatus]
	and	eax,[regIntMask]
	jz	short loc_4
	mov	gs:[Reg.ASR2],eax

loc_1:
	mov	[bp-2],ax

	mov	ax,TCI or TDU or TJT or TUF or GPTT
	test	word ptr [bp-2],ax
	jz	short loc_2
	not	ax
lock	and	word ptr [regIntStatus],ax
	call	_ServiceIntTx

loc_2:
	mov	ax,RCI or RDU or RWT or GPTT
	cmp	[Indication],0		; rx enable
	jnz	short loc_3
	test	word ptr [bp-2],ax
	jz	short loc_3
	not	ax
lock	and	word ptr [regIntStatus],ax
	call	_ServiceIntRx

loc_3:
lock	btr	[drvflags],df_rxsp
	jnc	short loc_0
loc_4:
	leave
	retn
_hwServiceInt	endp

_hwCheckInt	proc	near
	mov	eax,gs:[Reg.ASR2]
	and	eax,not(ANISS or AAISS)
lock	or	[regIntStatus],eax
	mov	eax,[regIntStatus]
	test	eax,[regIntMask]
	setnz	al
	mov	ah,0
	retn
_hwCheckInt	endp

_hwEnableInt	proc	near
	mov	eax,[regIntMask]
	mov	gs:[Reg.AIER2],eax	; set IMR
	retn
_hwEnableInt	endp

_hwDisableInt	proc	near
	mov	gs:[Reg.AIER2],0	; clear IMR
	retn
_hwDisableInt	endp

_hwIntReq	proc	near
	mov	gs:[Reg.TMR],1		; one-shot timer 204us
	retn
_hwIntReq	endp

_hwEnableRxInd	proc	near
	push	eax
lock	or	[regIntMask],RCI or RDU or RWT
	cmp	[semInt],0
	jnz	short loc_1
	mov	eax,[regIntMask]
	mov	gs:[Reg.AIER2],eax
loc_1:
	pop	eax
	retn
_hwEnableRxInd	endp

_hwDisableRxInd	proc	near
	push	eax
lock	and	[regIntMask],not(RCI or RDU or RWT)
	cmp	[semInt],0
	jnz	short loc_1
	mov	eax,[regIntMask]
	mov	gs:[Reg.AIER2],eax
loc_1:
	pop	eax
	retn
_hwDisableRxInd	endp


_hwPollLink	proc	near
	call	_ChkLink
	test	al,MediaLink
	jz	short loc_0	; Link status change/down
	retn
loc_0:
	or	al,al
	mov	MediaLink,al
	jnz	short loc_1	; change into Link Active
	call	_ChkLink	; link down. check again.
	or	al,al
	mov	MediaLink,al
	jnz	short loc_1	; short time link down
	retn

loc_1:
	call	_GetPhyMode

	cmp	al,MediaSpeed
	jnz	short loc_2
	cmp	ah,MediaDuplex
	jnz	short loc_2
	cmp	dl,MediaPause
	jz	short loc_3
loc_2:
	mov	MediaSpeed,al
	mov	MediaDuplex,ah
	mov	MediaPause,dl
	call	_SetMacEnv
	push	offset semFlt
	call	_EnterCrit
	or	gs:[Reg.NAR],STx or SRx	; re-start tx/rx
	call	_LeaveCrit
	pop	dx
loc_3:
	retn
_hwPollLink	endp

_hwOpen		proc	near	; call in protocol bind process?
	call	_AutoNegotiate
	mov	MediaSpeed,al
	mov	MediaDuplex,ah
	mov	MediaPause,dl

	call	_SetMacEnv

	mov	bx,[TxFreeHead]
	mov	eax,[bx].txd.physadr
	mov	gs:[Reg.TDB],eax	; tx descriptor base
	mov	bx,[RxHead]
	mov	eax,[bx].rxd.physadr
	mov	gs:[Reg.RDB],eax	; rx descriptor base
loc_2:

	xor	edx,edx
	mov	eax,TCI or TDU or TJT or TUF or \
		 RCI or RDU or RWT or GPTT or ANISS or AAISS

	mov	[regIntStatus],edx
	mov	[regIntMask],eax
	dec	edx
	mov	gs:[Reg.SR],edx		; clear interrupt status
	mov	gs:[Reg.ASR2],edx	; clear assistant status 2
	mov	gs:[Reg.AIER2],eax	; enable interrupt

;	push	offset semFlt
;	call	_EnterCrit
	or	gs:[Reg.NAR],STx or SRx	; start tx/rx
;	pop	cx

	mov	ax,SUCCESS
	retn
_hwOpen		endp

_SetMacEnv	proc	near
	push	offset semFlt
	call	_EnterCrit
	call	_StopTxRx
	mov	eax,gs:[Reg.NAR]
	mov	ecx,gs:[Reg.CR]
	ror	eax,8
	mov	al,[cfgTxDRTH]
	mov	ah,0
	cmp	al,4			; store and forward?
	jnz	short loc_1
	mov	ax,(SF or TR) shr (6+8)
loc_1:
	shl	ax,6
	and	cl,not PAUSE

	cmp	[MediaDuplex],0		; half duplex?
	jnz	short loc_2
	cmp	[MediaSpeed],1		; 10base-t half duplex?
	ja	short loc_3
	jmp	short loc_4
loc_2:
	test	[MediaPause],2		; rx pause?
	jz	short loc_3
	or	cl,PAUSE
loc_3:
	or	ax,SQE shr 8
loc_4:
	rol	eax,8
	mov	gs:[Reg.NAR],eax
	mov	gs:[Reg.CR],ecx
	call	_LeaveCrit
	call	_SetSpeedStat
	pop	cx	; stack adjust
	retn
_SetMacEnv	endp


_StopTxRx	proc	near
	and	gs:[Reg.NAR],not (STx or SRx)
	mov	cx,384
loc_1:
	test	word ptr gs:[Reg.SR][2],highword (TS or RS)
	jz	short loc_2
	push	4
	call	__IODelayCnt
	pop	ax
	dec	cx
	jnz	short loc_1
loc_2:
	retn
_StopTxRx	endp

_StopRx		proc	near
	and	gs:[Reg.NAR],not SRx		; clear SR bit
	mov	cx,384
loc_1:
	test	word ptr gs:[Reg.SR][2],highword RS ; rx status is stop?
	jz	short loc_2
	push	4
	call	__IODelayCnt
	pop	ax
	dec	cx
	jnz	short loc_1
loc_2:
	retn
_StopRx		endp

_StopTx		proc	near
	and	gs:[Reg.NAR],not STx ; clear ST bit
	mov	cx,384
loc_1:
	test	word ptr gs:[Reg.SR][2],highword TS ; tx status is stop?
	jz	short loc_2
	push	4
	call	__IODelayCnt
	pop	ax
	dec	cx
	jnz	short loc_1
loc_2:
	retn
_StopTx		endp


_SetSpeedStat	proc	near
	mov	al,[MediaSpeed]
	mov	ah,0
	dec	ax
	jz	short loc_10M
	dec	ax
	jz	short loc_100M
;	dec	ax
;	jz	short loc_1G
	xor	ax,ax
	sub	cx,cx
	jmp	short loc_1
loc_10M:
	mov	cx,highword 10000000
	mov	ax,lowword  10000000
	jmp	short loc_1
loc_100M:
	mov	cx,highword 100000000
	mov	ax,lowword  100000000
;	jmp	short loc_1
loc_1G:
;	mov	cx,highword 1000000000
;	mov	ax,lowword  1000000000
loc_1:
	mov	word ptr [MacChar.linkspeed],ax
	mov	word ptr [MacChar.linkspeed][2],cx
	retn
_SetSpeedStat	endp


_ChkLink	proc	near
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	and	ax,miiBMSR_LinkStat
	add	sp,2*2
	shr	ax,2
	retn
_ChkLink	endp


_AutoNegotiate	proc	near
	enter	2,0
	push	0
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite		; clear ANEnable bit
	add	sp,3*2

	push	33
	call	_Delay1ms
;	push	miiBMCR_ANEnable or miiBMCR_RestartAN
	push	miiBMCR_ANEnable	; remove restart bit??
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite		; restart Auto-Negotiation
	add	sp,(1+3)*2

	mov	word ptr [bp-2],12*30	; about 12sec.
loc_1:
	push	33
	call	_Delay1ms
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,(1+2)*2
	test	ax,miiBMCR_RestartAN	; AN in progress?
	jz	short loc_2
	dec	word ptr [bp-2]
	jnz	short loc_1
	jmp	short loc_f
loc_2:
	push	33
	call	_Delay1ms
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,(1+2)*2
	test	ax,miiBMSR_ANComp	; AN Base Page exchange complete?
	jnz	short loc_3
	dec	word ptr [bp-2]
	jnz	short loc_2
	jmp	short loc_f
loc_3:
	push	33
	call	_Delay1ms
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,(1+2)*2
	test	ax,miiBMSR_LinkStat	; link establish?
	jnz	short loc_4
	dec	word ptr [bp-2]
	jnz	short loc_3
loc_f:
	xor	ax,ax			; AN failure.
	xor	dx,dx
	leave
	retn
loc_4:
	call	_GetPhyMode
	leave
	retn
_AutoNegotiate	endp

_GetPhyMode	proc	near
	push	miiANLPAR
	push	[PhyInfo.Phyaddr]
	call	_miiRead		; read base page
	add	sp,2*2
	mov	[PhyInfo.ANLPAR],ax

	test	[PhyInfo.BMSR],miiBMSR_ExtStat
	jz	short loc_2

	push	mii1KSTSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GSTSR],ax
;	shl	ax,2
;	and	ax,[PhyInfo.GSCR]
	shr	ax,2
	and	ax,[PhyInfo.GTCR]
;	test	ax,mii1KSCR_1KTFD
	test	ax,mii1KTCR_1KTFD
	jz	short loc_1
	mov	al,3			; media speed - 1000Mb
	mov	ah,1			; media duplex - full
	jmp	short loc_p
loc_1:
;	test	ax,mii1KSCR_1KTHD
	test	ax,mii1KTCR_1KTHD
	jz	short loc_2
	mov	al,3			; 1000Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_2:
	mov	ax,[PhyInfo.ANAR]
	and	ax,[PhyInfo.ANLPAR]
	test	ax,miiAN_100FD
	jz	short loc_3
	mov	al,2			; 100Mb
	mov	ah,1			; full duplex
	jmp	short loc_p
loc_3:
	test	ax,miiAN_100HD
	jz	short loc_4
	mov	al,2			; 100Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_4:
	test	ax,miiAN_10FD
	jz	short loc_5
	mov	al,1			; 10Mb
	mov	ah,1			; full duplex
	jmp	short loc_p
loc_5:
	test	ax,miiAN_10HD
	jz	short loc_e
	mov	al,1			; 10Mb
	mov	ah,0			; half duplex
	jmp	short loc_p
loc_e:
	xor	ax,ax
	sub	dx,dx
	retn
loc_p:
	cmp	ah,1			; full duplex?
	mov	dh,0
	jnz	short loc_np
	mov	cx,[PhyInfo.ANLPAR]
	test	cx,miiAN_PAUSE		; symmetry
	mov	dl,3			; tx/rx pause
	jnz	short loc_ex
	test	cx,miiAN_ASYPAUSE	; asymmetry
	mov	dl,2			; rx pause
	jnz	short loc_ex
loc_np:
	mov	dl,0			; no pause
loc_ex:
	retn
_GetPhyMode	endp


_ResetPhy	proc	near
	enter	2,0
	call	_miiReset	; Reset Interface
	push	miiPHYID2
	push	1		; phyaddr 1
	call	_miiRead
	add	sp,2*2
	or	ax,ax		; ID2 = 0
	jz	short loc_1
	inc	ax		; ID2 = -1
	jnz	short loc_2
loc_1:
	mov	ax,HARDWARE_FAILURE
	leave
	retn
loc_2:
	mov	[PhyInfo.Phyaddr],1
	push	miiBMCR_Reset
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite	; Reset PHY
	add	sp,3*2

	push	1536		; wait for about 1.5sec.
	call	_Delay1ms
	pop	ax

	call	_miiReset	; interface reset again
	mov	word ptr [bp-2],64  ; about 2sec.
loc_3:
	push	miiBMCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	test	ax,miiBMCR_Reset
	jz	short loc_4
	push	33
	call	_Delay1ms	; wait reset complete.
	pop	ax
	dec	word ptr [bp-2]
	jnz	short loc_3
	jmp	short loc_1	; PHY Reset Failure
loc_4:
	push	miiBMSR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.BMSR],ax
	push	miiANAR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.ANAR],ax
	test	[PhyInfo.BMSR],miiBMSR_ExtStat
	jz	short loc_5	; extended status exist?
	push	mii1KTCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GTCR],ax
	push	mii1KSCR
	push	[PhyInfo.Phyaddr]
	call	_miiRead
	add	sp,2*2
	mov	[PhyInfo.GSCR],ax
	xor	cx,cx
	test	ax,mii1KSCR_1KTFD or mii1KSCR_1KXFD
	jz	short loc_41
	or	cx,mii1KTCR_1KTFD
loc_41:
			; kill 1000BASE half-duplex advertisement
;	test	ax,mii1KSCR_1KTHD or mii1KSCR_1KXHD
;	jz	short loc_42
;	or	cx,mii1KTCR_1KTHD
loc_42:
	mov	ax,[PhyInfo.GTCR]
	and	ax,not (mii1KTCR_MSE or mii1KTCR_Port or \
		  mii1KTCR_1KTFD or mii1KTCR_1KTHD)
	or	ax,cx
	mov	[PhyInfo.GTCR],ax
	push	ax
	push	mii1KTCR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite
	add	sp,2*2
loc_5:
	mov	ax,[PhyInfo.BMSR]
	mov	cx,miiAN_PAUSE
	test	ax,miiBMSR_100FD
	jz	short loc_61
	or	cx,miiAN_100FD
loc_61:
	test	ax,miiBMSR_100HD
	jz	short loc_62
	or	cx,miiAN_100HD
loc_62:
	test	ax,miiBMSR_10FD
	jz	short loc_63
	or	cx,miiAN_10FD
loc_63:
	test	ax,miiBMSR_10HD
	jz	short loc_64
	or	cx,miiAN_10HD
loc_64:
	mov	ax,[PhyInfo.ANAR]
	and	ax,not (miiAN_ASYPAUSE + miiAN_T4 + \
	  miiAN_100FD + miiAN_100HD + miiAN_10FD + miiAN_10HD)
	or	ax,cx
	mov	[PhyInfo.ANAR],ax
	push	ax
	push	miiANAR
	push	[PhyInfo.Phyaddr]
	call	_miiWrite
	add	sp,3*2
	mov	ax,SUCCESS
	leave
	retn
_ResetPhy	endp


_hwUpdateMulticast	proc	near
	enter	2,0
	push	offset semFlt
	call	_EnterCrit

	push	gs:[Reg.NAR]
	call	_StopRx

	mov	bx,offset regHashTable
	mov	cx,MCSTList.curnum
	xor	eax,eax
	mov	[bx],eax
	mov	[bx+4],eax	; clear hash table

	dec	cx
	jl	short loc_2
	mov	[bp-2],cx
loc_1:
	mov	ax,[bp-2]
	shl	ax,4		; 16bytes
	add	ax,offset MCSTList.multicastaddr1
	push	ax
	call	_CRC32
	and	ax,3fh		; the 6 least significant bits
	pop	dx	; stack adjust
	mov	bx,ax
	mov	cx,ax
	shr	bx,4
	and	cl,0fh		; the bit index in word
	mov	ax,1
	add	bx,bx		; the word index (2byte)
	shl	ax,cl
	or	word ptr regHashTable[bx],ax
	dec	word ptr [bp-2]
	jge	short loc_1
loc_2:
	mov	eax,dword ptr [regHashTable]
	mov	ecx,dword ptr [regHashTable][4]
	mov	gs:[Reg.MAR0],eax
	mov	gs:[Reg.MAR1],ecx
	pop	gs:[Reg.NAR]

	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	leave
	retn
_hwUpdateMulticast	endp

_CRC32		proc	near
POLYNOMIAL_be   equ  04C11DB7h
POLYNOMIAL_le   equ 0EDB88320h

	push	bp
	mov	bp,sp

	push	si
	push	di
	or	ax,-1
	mov	bx,[bp+4]
	mov	ch,3
	cwd

loc_1:
	mov	bp,[bx]
	mov	cl,10h
	inc	bx
loc_2:
IF 0
		; big endian

	ror	bp,1
	mov	si,dx
	xor	si,bp
	shl	ax,1
	rcl	dx,1
	sar	si,15
	mov	di,si
	and	si,highword POLYNOMIAL_be
	and	di,lowword POLYNOMIAL_be
ELSE
		; litte endian
	mov	si,ax
	ror	bp,1
	ror	si,1
	shr	dx,1
	rcr	ax,1
	xor	si,bp
	sar	si,15
	mov	di,si
	and	si,highword POLYNOMIAL_le
	and	di,lowword POLYNOMIAL_le
ENDIF
	xor	dx,si
	xor	ax,di
	dec	cl
	jnz	short loc_2
	inc	bx
	dec	ch
	jnz	short loc_1
	push	dx
	push	ax
	pop	eax
	pop	di
	pop	si
	pop	bp
	retn
_CRC32		endp

_hwUpdatePktFlt	proc	near
	push	offset semFlt
	call	_EnterCrit

	push	gs:[Reg.NAR]		; backup current status
	call	_StopRx
	pop	eax
	mov	cx,[MacStatus.sstRxFilter]
	mov	dx,[cfgRxAcErr]
	mov	bx,rxCE or rxRF

	and	al,not (MM or PR or PB)
	and	dx,bx
	test	cl,mask fltprms
	jz	short loc_1
	or	al,MM or PR		; set promiscous
loc_1:
	xor	dx,bx
	jz	short loc_2
	or	al,PB			; pass bad packet
loc_2:
	mov	gs:[Reg.NAR],eax

	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	retn
_hwUpdatePktFlt	endp

_hwSetMACaddr	proc	near
	push	offset semFlt
	call	_EnterCrit

	push	gs:[Reg.NAR]
	call	_StopRx		; writeable only when rx is stopped.

	mov	bx,offset MacChar.mctcsa
	mov	ax,[bx]
	or	ax,[bx+2]
	or	ax,[bx+4]
	jnz	short loc_1
	mov	bx,offset MacChar.mctpsa
loc_1:
	xor	eax,eax
	mov	ecx,[bx]
	mov	ax,[bx+4]
	mov	gs:[Reg.PAR0],ecx
	mov	gs:[Reg.PAR1],eax

	pop	gs:[Reg.NAR]

	call	_LeaveCrit
	pop	cx
	mov	ax,SUCCESS
	retn
_hwSetMACaddr	endp

_hwUpdateStat	proc	near
	push	offset semStat
	call	_EnterCrit

	mov	eax,gs:[Reg.LPC]
	add	[MacStatus.rxframebuf],eax

	call	_LeaveCrit
	pop	ax
	retn
_hwUpdateStat	endp

_hwClearStat	proc	near
	mov	eax,gs:[Reg.LPC]
	retn
_hwClearStat	endp

_hwClose	proc	near
	push	offset semFlt
	call	_EnterCrit
	mov	gs:[Reg.AIER2],eax
	mov	gs:[Reg.IER],eax
	mov	[regIntMask],eax
	call	_StopTxRx
	or	eax,-1
	mov	gs:[Reg.ASR2],eax
	mov	gs:[Reg.SR],eax

	call	_LeaveCrit
	pop	dx

	mov	ax,SUCCESS
	retn
_hwClose	endp

_hwReset	proc	near	; call in bind process
	enter	6,0

	xor	eax,eax
	mov	gs:[Reg.IER],eax	; clear IER
	mov	gs:[Reg.AIER2],eax	; clear Assistant IER
	mov	gs:[Reg.CR],eax		; clear software interrupt
	dec	eax
	mov	gs:[Reg.SR],eax		; clear SR
	mov	gs:[Reg.ASR2],eax	; clear Assistant SR
	mov	gs:[Reg.WCSR],WFR or MPR or LSC	; clear WOL
	neg	eax
	mov	gs:[Reg.PAR],eax	; reset

	mov	word ptr [bp-2],32
loc_1:
	push	96
	call	_Delay1ms
	mov	eax,gs:[Reg.PAR]
	pop	cx
	test	ax,SWR
	jz	short loc_2
	dec	word ptr [bp-2]
	jnz	short loc_1
	mov	ax,HARDWARE_FAILURE
	leave
	retn

loc_2:
	xor	eax,eax
	mov	gs:[Reg.IER],eax	; clear IER again
	mov	gs:[Reg.AIER2],eax	; clear Assistant IER again
	mov	gs:[Reg.CR],eax		; clear SINT again
	mov	gs:[Reg.RDB],eax	; clear Rx Desc pointer
	mov	gs:[Reg.TDB],eax	; clear Tx Desc pointer

	mov	al,[cfgTAP]
	mov	dl,[CacheLine]
	mov	ah,0
	shl	dx,14
	jz	short loc_5		; cache line size is zero.
	cmp	[cfgPCIMWI],0
	jz	short loc_3
	or	ax,highword(MWIE) shr 1
loc_3:
	cmp	[cfgPCIMRL],0
	jz	short loc_4
	or	ax,highword(MRLE) shr 1
loc_4:
	cmp	[cfgPCIMRM],0
	jz	short loc_5
	or	ax,highword(MRME) shr 1
loc_5:
	mov	dl,[cfgPCIBAP]
	or	dh,[cfgMXDMA]
	shl	dl,1
	shl	eax,17
	mov	ax,dx
	mov	gs:[Reg.PAR],eax

	mov	al,[cfgTBCNT]
	mov	ah,0
	shl	eax,16
	mov	ax,[cfgTTO]
	mov	gs:[Reg.TXBR],eax

	xor	eax,eax
	cmp	[cfgMAXFRAMESIZE],2048-4
	jc	short loc_6
	or	al,JBD or RWD
loc_6:
	mov	gs:[Reg.WTMR],eax

	mov	ecx,gs:[RegCmt.PID1]	; If the chip is centaur, 
	mov	eax,gs:[RegCmt.PID2]	; these registers are zero.
	or	ax,cx			; If comet, PHY IDs.
	setz	[ChipType]
	jnz	short loc_7		; comet

	xor	eax,eax
	mov	gs:[Reg.UAR0],eax	; clear Unicast Hash table (AN983B)
	mov	gs:[Reg.UAR1],eax	; or reserved register (AN983)
	or	gs:[Reg.OPR],7		; select internal PHY(single mode)
loc_7:

	call	_eepCheckType
	cmp	ax,SUCCESS
	jnz	short loc_ex

	push	17h			; CR bit 31-16
	call	_eepRead
	shl	eax,16
	mov	al,[cfgRxDRTH]
	shl	ax,2
	or	ax,RTE or ATUR
	mov	gs:[Reg.CR],eax

	push	4			; MAC Address
	call	_eepRead
	mov	[bp-6],ax
	push	5
	call	_eepRead
	mov	[bp-4],ax
	push	6
	call	_eepRead
	mov	[bp-2],ax

	push	offset semFlt
	call	_EnterCrit
	mov	ax,[bp-6]
	mov	cx,[bp-4]
	mov	dx,[bp-2]
	mov	word ptr MacChar.mctpsa,ax	; parmanent
	mov	word ptr MacChar.mctpsa[2],cx
	mov	word ptr MacChar.mctpsa[4],dx
	mov	word ptr MacChar.mctcsa,ax	; current
	mov	word ptr MacChar.mctcsa[2],cx
	mov	word ptr MacChar.mctcsa[4],dx
	mov	word ptr MacChar.mctVendorCode,ax ; vendor
	mov	byte ptr MacChar.mctVendorCode[2],cl
	call	_LeaveCrit
	add	sp,5*2
	call	_hwSetMACaddr		; update PAR0,1

	call	_ResetPhy
loc_ex:
	leave
	retn
_hwReset	endp


; USHORT miiRead( UCHAR phyaddr, UCHAR phyreg)
_miiRead	proc	near
	push	bp
	mov	bp,sp
	push	offset semMii
	call	_EnterCrit

	cmp	[ChipType],1
	jc	near ptr loc_cmt

	mov	bx,offset Reg.SPR +2	; centaur
	push	1
	mov	word ptr gs:[bx-2],0	; clear eeprom select

	mov	byte ptr gs:[bx],highword MDO	; idle
	call	__IODelayCnt
	mov	byte ptr gs:[bx],highword (MDO or MDC)
	call	__IODelayCnt

	mov	dl,[bp+4]	; physaddr (5bit)
	mov	cl,[bp+6]	; phyreg   (5bit)
	shl	dx,5
	and	cl,1fh
	and	dh,3
	or	dl,cl
;	or	dx,0110b shl 10
	or	dh,0110b shl 2	; start(01) + opcode(10)
	mov	cx,14-1

loc_1:
	mov	al,0
	bt	dx,cx
	rcl	al,2
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	byte ptr gs:[bx],highword MMC	; TA (z0)
	call	__IODelayCnt
	mov	byte ptr gs:[bx],highword (MMC or MDC)
	call	__IODelayCnt

	mov	cx,16
loc_2:
	mov	byte ptr gs:[bx],highword MMC
	call	__IODelayCnt
	mov	byte ptr gs:[bx],highword (MMC or MDC)
	call	__IODelayCnt
	mov	dl,gs:[bx]
	shr	dx,4		; MDI
	rcl	ax,1
	dec	cx
	jnz	short loc_2

	mov	byte ptr gs:[bx],highword MDO	; idle
	call	__IODelayCnt
	mov	byte ptr gs:[bx],highword (MDO or MDC)
	call	__IODelayCnt

	pop	cx	; stack adjust
loc_ex:
	call	_LeaveCrit
	pop	cx	; stack adjust
	pop	bp
	retn

loc_cmt:
	mov	ax,[bp+6]
	mov	bx,offset RegCmt.XCR
	cmp	ax,6
	jna	short loc_3
	sub	ax,10h
	jc	short loc_ex
	mov	bx,offset RegCmt.XMC
	cmp	ax,3
	ja	short loc_ex
loc_3:
	shl	ax,2
	add	bx,ax
	mov	eax,gs:[bx]
	jmp	short loc_ex

_miiRead	endp

; VOID miiWrite( UCHAR phyaddr, UCHAR phyreg, USHORT value)
_miiWrite	proc	near
	push	bp
	mov	bp,sp
	push	offset semMii
	call	_EnterCrit

	cmp	[ChipType],1
	jc	near ptr loc_cmt

	mov	bx,offset Reg.SPR +2	; centaur
	push	1
	mov	word ptr gs:[bx-2], 0	; clear EEPROM select

	mov	al,highword MDO
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al		; idle
	call	__IODelayCnt

	mov	dl,[bp+4]	; physaddr (5bit)
	mov	cl,[bp+6]	; phyreg   (5bit)
	shl	dx,5
	and	cl,1fh
	and	dx,3E0h
	or	dl,cl
	or	dh,0101b shl 2	; start(01) + opcode(01)
	mov	cx,14-1

loc_1:
	mov	al,0
	bt	dx,cx
	rcl	al,2		; MDO
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	al,highword MDO		; TA (10)
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al
	call	__IODelayCnt
	mov	al,0
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al
	call	__IODelayCnt

	mov	dx,[bp+8]
	mov	cx,15
loc_2:
	mov	al,0
	bt	dx,cx
	rcl	al,2
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al
	call	__IODelayCnt
	dec	cx
	jge	short loc_2

	mov	al,highword MDO
	mov	gs:[bx],al
	call	__IODelayCnt
	or	al,highword MDC
	mov	gs:[bx],al		; idle
	call	__IODelayCnt

	pop	cx	;stack adjust
loc_ex:
	call	_LeaveCrit
	leave
	retn

loc_cmt:				; comet
	mov	cx,[bp+6]
	mov	ax,[bp+8]
	mov	bx,offset RegCmt.XCR	; 0-6
	cmp	cx,6
	jna	short loc_3
	sub	cx,10h
	jc	short loc_ex
	mov	bx,offset RegCmt.XMC	; 10-13
	cmp	cx,3
	ja	short loc_ex
loc_3:
	shl	cx,2
	add	bx,cx
	mov	gs:[bx],eax
	jmp	short loc_ex

_miiWrite	endp

; VOID miiReset( VOID )
_miiReset	proc	near
	cmp	[ChipType],1
	jc	short loc_ex		; do nothing if comet.

	push	offset semMii
	call	_EnterCrit
	mov	bx,offset Reg.SPR +2
	push	1
	mov	word ptr gs:[bx-2], 0	; clear EEPROM select

	mov	cx,32			; 32clocks
loc_1:
	mov	byte ptr gs:[bx],highword MDO	; high
	call	__IODelayCnt
	mov	byte ptr gs:[bx],highword (MDO or MDC)
	call	__IODelayCnt
	loop	short loc_1
	pop	cx	; stack adjust
	call	_LeaveCrit
	pop	cx	; stack adjust
loc_ex:
	retn
_miiReset	endp

; USHORT eepRead( UCHAR addr )
_eepRead	proc	near
	push	bp
	mov	bp,sp
	mov	bx,offset Reg.SPR


	mov	ax,SRC or SRS
	mov	gs:[bx],ax	; chip select - low
;	push	1
	push	4
	call	__IODelayCnt
	or	ax,SCLK
	mov	gs:[bx],al
	call	__IODelayCnt

	mov	dl,[bp+4]		; address
	mov	dh,0
	mov	ax,(1 + 2 + 6) -1	; length
	mov	bp,110b shl 6		; start + read
	mov	cl,[eepType]		; 0:46 1:56 2:66
	cmp	cl,2
	ja	short loc_0		; unknown type?? assume 93C46
	shl	bp,cl
	add	al,cl
loc_0:
	or	dx,bp
	mov	cx,ax
loc_1:
	xor	ax,ax
	bt	dx,cx
	rcl	ax,3
	or	ax,SWC or SRS or SCS
	mov	gs:[bx],ax
	call	__IODelayCnt
	or	ax,SCLK
	mov	gs:[bx],ax
	call	__IODelayCnt
	dec	cx
	jge	short loc_1

	mov	cx,16
	xor	dx,dx
loc_2:
	mov	ax,SRC or SRS or SCS
	mov	gs:[bx],ax
	call	__IODelayCnt
	or	ax,SCLK
	mov	gs:[bx],ax
	call	__IODelayCnt
	mov	ax,gs:[bx]
	shr	ax,4
	rcl	dx,1
	dec	cx
	jnz	short loc_2

	mov	ax,SRC or SRS		; chip select low
	mov	gs:[bx],ax
	call	__IODelayCnt
	or	ax,SCLK
	call	__IODelayCnt

	pop	cx	; stack adjust
	pop	bp
	mov	ax,dx
	retn
_eepRead	endp

; USHORT eepCheckType( VOID )
_eepCheckType	proc	near
	mov	al,2			; 93c66
loc_1:
	mov	[eepType],al
	push	0
	call	_eepRead
	pop	cx		; stack adjust
	cmp	ax,981h			; comet AL981
	jz	short loc_3
	cmp	ax,985h			; centaur AN983
	jz	short loc_3
	cmp	ax,1985h		; AN985
	jz	short loc_3
	mov	al,[eepType]
	dec	al
	jge	short loc_1
loc_2:
	mov	ax,HARDWARE_FAILURE	; unknown type
	retn
loc_3:
	push	1
	call	_eepRead
	pop	cx		; stack adjust
	cmp	ax,0002h		; format version Maj=02, Min=00
	jz	short loc_4
	cmp	ax,0001h		; Maj=01 (comet only)
	jnz	short loc_2
loc_4:
	mov	ax,SUCCESS
	retn
_eepCheckType	endp

; void _IODelayCnt( USHORT count )
__IODelayCnt	proc	near
	push	bp
	mov	bp,sp
	push	cx
	mov	bp,[bp+4]
loc_1:
	mov	cx,offset DosIODelayCnt
	dec	bp
	loop	$
	jnz	short loc_1
	pop	cx
	pop	bp
	retn
__IODelayCnt	endp


_TEXT	ends
end
