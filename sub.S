/*------------------------------------------------------------

	DISK II Emulator Farmware (2 of 2) for ATMEGA328P

	version 2.2	2013.10.25 by Koichi Nishida
	require version 2.0 hardware

	Copyright 2012 Koichi NISHIDA
	email to Koichi NISHIDA: tulip-house@msf.biglobe.ne.jp
	
------------------------------------------------------------*/

.equ PINB, 0x03
.equ DDRB, 0x04
.equ PORTB, 0x05
.equ PINC, 0x06
.equ DDRC, 0x07
.equ PORTC, 0x08
.equ PIND, 0x09
.equ DDRD, 0x0a
.equ PORTD, 0x0b
.equ SREG, 0x3f

.global __vector_1
.global __vector_14
.global wait5

.global readPulse
.global bitByte
.global sector
.global prepare
.global writeData
.global buffering
.global writePtr

.func wait5
wait5:
	ldi r18,24
wait51:
	nop
	dec r18
	brne wait51
	sbiw r24,1
	brne wait5
	ret
.endfunc	
	
.func wait1
wait1:
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	nop		; 1
	ret		; 4
.endfunc

.func __vector_14
__vector_14:
	push	r26
	in		r26, SREG
	push	r26
	lds		r26,readPulse
	ori		r26,0b00100000
	out 	PORTC,r26
	
	push	r27				; 2
	push	r18				; 2
	ldi		r18,0			; 1
	rcall	wait1			; 20
	lds		r27,prepare		; 2
	and		r27,r27			; 1
	breq 	NOT_PREPARE		; 1/2
	ldi		r26,0b00100000	; 1
	
	out 	PORTC,r26		; 1
	sts		readPulse,r18
	pop		r18
	pop		r27
	pop		r26
	out		SREG,r26	
	pop		r26
	reti
NOT_PREPARE:
	ldi		r26,0b00100000
	out 	PORTC,r26
	ldi		r26,0b00111000
	out		PORTB,r26
	in		r26,PINB
	andi	r26,0b00010000
	mov		r18,r26
	ldi		r26,0b00011000
	out		PORTB,r26
	lds		r26,bitbyte
	lds		r27,(bitbyte+1)
	adiw	r26,1
	sts		bitbyte,r26
	sts		(bitbyte+1),r27
	cpi		r26,((402*8)%256)
	brne	LBL1
	cpi		r27,((402*8)/256)
	brne	LBL1
	; set prepare flag
	ldi		r26,1
	sts		prepare,r26
	; discard 112 byte (including CRC 2 byte)
	push	r28
	push	r29
	ldi		r28,112
	ldi		r27,0b00111000
	ldi		r29,0b00011000
DSC_LP2:
	ldi		r26,8
DSC_LP1:
	out		PORTB,r27
	out		PORTB,r29
	dec		r26
	brne	DSC_LP1
	dec		r28
	brne	DSC_LP2
	pop		r29
	pop		r28
LBL1:
	sts		readPulse,r18
	pop		r18
	pop		r27
	pop		r26
	out		SREG,r26	
	pop		r26
	reti
.endfunc

.func __vector_1
__vector_1:
	push	r18			; 1
	in		r18, SREG	; 1
	push	r18			; 1
	sbic	PIND,7
	rjmp	NOT_ENABLE
	push	r19			; 2
	lds		r19,magState; 2
WLP8:
	; wait start bit 1
	in		r18,PINB	; 1
	andi	r18,2		; 1
	eor		r18,r19		; 1
	breq	WLP8		; 2/1
	in		r18,PINB	; 1
	andi	r18,2		; 1
	sts		magState,r18; 2
	ldi		r18, 8		; 1
WLP9:
	dec		r18			; 1
	brne	WLP9		; 2
	nop					; 1
	push	r20			; 2
	push	r21			; 2
	push	r22			; 2
	push	r23			; 2
	push	r24			; 2
	push	r30			; 2
	push	r31			; 2
	ldi		r22,0		; 1 start storing
	lds		r30,(writePtr)
	lds		r31,(writePtr+1)
	ldi		r19,lo8(349) ;1
	ldi		r20,hi8(349) ;1 
	rjmp	ENTR		; 2
WLP2:
	lds		r21,magState; 2
WLP6:
	; wait start bit 1
	in		r23,PINB	; 1
	andi	r23,2		; 1
	eor		r23,r21		; 1
	breq	WLP6		; 2/1
	in		r23,PINB	; 1
	andi	r23,2		; 1
	sts		magState,r23; 2
	ldi		r23, 14		; 1
WLP7:
	dec		r23			; 1
	brne	WLP7		; 2
ENTR:
	ldi		r18,7		; 1
	ldi		r24,1		; 1
WLP1:
	in		r23,PIND	; 1
	andi	r23,4		; 1
	brne	WRITE_END	; 1	
	nop					; 1	
	ldi		r23, 30		; 1
WLP3:
	dec		r23			; 1
	brne	WLP3		; 2
WLP5:
	in		r23,PINB 	; 1
	andi	r23,2		; 1
	lds		r21,magState; 2
	sts		magState,r23; 2
	eor		r23,r21		; 1
	lsr		r23			; 1
	nop					; 1
	lsl		r24			; 1
	or		r24,r23		; 1
	dec		r18			; 1
	brne	WLP1		; 2/1
	cpi		r24,0xD5	; 1
	brne	NOT_START	; 2/1
	ldi		r22,1		; 1
NOT_START:
	cpi		r22,0		; 1
	breq	WLP2		; 1
	st		Z+,r24		; 2
	subi	r19,1		; 1
	sbci	r20,0		; 1
	brne	WLP2		; 2/1
WRITE_END:
	ldi		r21,1
	sts		doBuffering,r21
	pop		r31
	pop		r30
	pop		r24
	pop		r23
	pop		r22
	pop		r21
	pop		r20
	pop		r19
NOT_ENABLE:
	pop		r18
	out		SREG,r18	
	pop		r18
	reti
.endfunc

