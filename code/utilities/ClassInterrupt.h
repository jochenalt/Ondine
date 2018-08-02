/*
 * Util.h
 *
 *  Created on: 29.07.2018
 *      Author: JochenAlt
 */

#ifndef CLASS_INTERRUPT_H_
#define CLASS_INTERRUPT_H_

#include <Arduino.h>

typedef struct {
	void* object;
	void (*function)(void*);
} InterruptState;

extern InterruptState interruptArgs[];

static void update(InterruptState state) {
	state.function(state.object);
}

#ifdef CORE_INT0_PIN
static void class_isr0(void) { update(interruptArgs[0]); }
#endif
#ifdef CORE_INT1_PIN
static void class_isr1(void) { update(interruptArgs[1]); }
#endif
#ifdef CORE_INT2_PIN
static void class_isr2(void) { update(interruptArgs[2]); }
#endif
#ifdef CORE_INT3_PIN
static void class_isr3(void) { update(interruptArgs[3]); }
#endif
#ifdef CORE_INT4_PIN
static void class_isr4(void) { update(interruptArgs[4]); }
#endif
#ifdef CORE_INT5_PIN
static void class_isr5(void) { update(interruptArgs[5]); }
#endif
#ifdef CORE_INT6_PIN
static void class_isr6(void) { update(interruptArgs[6]); }
#endif
#ifdef CORE_INT7_PIN
static void class_isr7(void) { update(interruptArgs[7]); }
#endif
#ifdef CORE_INT8_PIN
static void class_isr8(void) { update(interruptArgs[8]); }
#endif
#ifdef CORE_INT9_PIN
static void class_isr9(void) { update(interruptArgs[9]); }
#endif
#ifdef CORE_INT10_PIN
static void class_isr10(void) { update(interruptArgs[10]); }
#endif
#ifdef CORE_INT11_PIN
static void class_isr11(void) { update(interruptArgs[11]); }
#endif
#ifdef CORE_INT12_PIN
static void class_isr12(void) { update(interruptArgs[12]); }
#endif
#ifdef CORE_INT13_PIN
static void class_isr13(void) { update(interruptArgs[13]); }
#endif
#ifdef CORE_INT14_PIN
static void class_isr14(void) { update(interruptArgs[14]); }
#endif
#ifdef CORE_INT15_PIN
static void class_isr15(void) { update(interruptArgs[15]); }
#endif
#ifdef CORE_INT16_PIN
static void class_isr16(void) { update(interruptArgs[16]); }
#endif
#ifdef CORE_INT17_PIN
static void class_isr17(void) { update(interruptArgs[17]); }
#endif
#ifdef CORE_INT18_PIN
static void class_isr18(void) { update(interruptArgs[18]); }
#endif
#ifdef CORE_INT19_PIN
static void class_isr19(void) { update(interruptArgs[19]); }
#endif
#ifdef CORE_INT20_PIN
static void class_isr20(void) { update(interruptArgs[20]); }
#endif
#ifdef CORE_INT21_PIN
static void class_isr21(void) { update(interruptArgs[21]); }
#endif
#ifdef CORE_INT22_PIN
static void class_isr22(void) { update(interruptArgs[22]); }
#endif
#ifdef CORE_INT23_PIN
static void class_isr23(void) { update(interruptArgs[23]); }
#endif
#ifdef CORE_INT24_PIN
static void class_isr24(void) { update(interruptArgs[24]); }
#endif
#ifdef CORE_INT25_PIN
static void class_isr25(void) { update(interruptArgs[25]); }
#endif
#ifdef CORE_INT26_PIN
static void class_isr26(void) { update(interruptArgs[26]); }
#endif
#ifdef CORE_INT27_PIN
static void class_isr27(void) { update(interruptArgs[27]); }
#endif
#ifdef CORE_INT28_PIN
static void class_isr28(void) { update(interruptArgs[28]); }
#endif
#ifdef CORE_INT29_PIN
static void class_isr29(void) { update(interruptArgs[29]); }
#endif
#ifdef CORE_INT30_PIN
static void class_isr30(void) { update(interruptArgs[30]); }
#endif
#ifdef CORE_INT31_PIN
static void class_isr31(void) { update(interruptArgs[31]); }
#endif
#ifdef CORE_INT32_PIN
static void class_isr32(void) { update(interruptArgs[32]); }
#endif
#ifdef CORE_INT33_PIN
static void class_isr33(void) { update(interruptArgs[33]); }
#endif
#ifdef CORE_INT34_PIN
static void class_isr34(void) { update(interruptArgs[34]); }
#endif
#ifdef CORE_INT35_PIN
static void class_isr35(void) { update(interruptArgs[35]); }
#endif
#ifdef CORE_INT36_PIN
static void class_isr36(void) { update(interruptArgs[36]); }
#endif
#ifdef CORE_INT37_PIN
static void class_isr37(void) { update(interruptArgs[37]); }
#endif
#ifdef CORE_INT38_PIN
static void class_isr38(void) { update(interruptArgs[38]); }
#endif
#ifdef CORE_INT39_PIN
static void class_isr39(void) { update(interruptArgs[39]); }
#endif
#ifdef CORE_INT40_PIN
static void class_isr40(void) { update(interruptArgs[40]); }
#endif
#ifdef CORE_INT41_PIN
static void class_isr41(void) { update(interruptArgs[41]); }
#endif
#ifdef CORE_INT42_PIN
static void class_isr42(void) { update(interruptArgs[42]); }
#endif
#ifdef CORE_INT43_PIN
static void class_isr43(void) { update(interruptArgs[43]); }
#endif
#ifdef CORE_INT44_PIN
static void class_isr44(void) { update(interruptArgs[44]); }
#endif
#ifdef CORE_INT45_PIN
static void class_isr45(void) { update(interruptArgs[45]); }
#endif
#ifdef CORE_INT46_PIN
static void class_isr46(void) { update(interruptArgs[46]); }
#endif
#ifdef CORE_INT47_PIN
static void class_isr47(void) { update(interruptArgs[47]); }
#endif
#ifdef CORE_INT48_PIN
static void class_isr48(void) { update(interruptArgs[48]); }
#endif
#ifdef CORE_INT49_PIN
static void class_isr49(void) { update(interruptArgs[49]); }
#endif
#ifdef CORE_INT50_PIN
static void class_isr50(void) { update(interruptArgs[50]); }
#endif
#ifdef CORE_INT51_PIN
static void class_isr51(void) { update(interruptArgs[51]); }
#endif
#ifdef CORE_INT52_PIN
static void class_isr52(void) { update(interruptArgs[52]); }
#endif
#ifdef CORE_INT53_PIN
static void class_isr53(void) { update(interruptArgs[53]); }
#endif
#ifdef CORE_INT54_PIN
static void class_isr54(void) { update(interruptArgs[54]); }
#endif
#ifdef CORE_INT55_PIN
static void class_isr55(void) { update(interruptArgs[55]); }
#endif
#ifdef CORE_INT56_PIN
static void class_isr56(void) { update(interruptArgs[56]); }
#endif
#ifdef CORE_INT57_PIN
static void class_isr57(void) { update(interruptArgs[57]); }
#endif
#ifdef CORE_INT58_PIN
static void class_isr58(void) { update(interruptArgs[58]); }
#endif
#ifdef CORE_INT59_PIN
static void class_isr59(void) { update(interruptArgs[59]); }
#endif



static uint8_t attachInterruptClass(uint8_t pin, void* object, void (*function)(void*)) {
	InterruptState state;
	state.function = function;
	state.object = object;

		switch (pin) {
		#ifdef CORE_INT0_PIN
			case CORE_INT0_PIN:
				interruptArgs[0] = state;
				attachInterrupt(0, class_isr0, CHANGE);
				break;
		#endif
		#ifdef CORE_INT1_PIN
			case CORE_INT1_PIN:
				interruptArgs[1] = state;
				attachInterrupt(1, class_isr1, CHANGE);
				break;
		#endif
		#ifdef CORE_INT2_PIN
			case CORE_INT2_PIN:
				interruptArgs[2] = state;
				attachInterrupt(2, class_isr2, CHANGE);
				break;
		#endif
		#ifdef CORE_INT3_PIN
			case CORE_INT3_PIN:
				interruptArgs[3] = state;
				attachInterrupt(3, class_isr3, CHANGE);
				break;
		#endif
		#ifdef CORE_INT4_PIN
			case CORE_INT4_PIN:
				interruptArgs[4] = state;
				attachInterrupt(4, class_isr4, CHANGE);
				break;
		#endif
		#ifdef CORE_INT5_PIN
			case CORE_INT5_PIN:
				interruptArgs[5] = state;
				attachInterrupt(5, class_isr5, CHANGE);
				break;
		#endif
		#ifdef CORE_INT6_PIN
			case CORE_INT6_PIN:
				interruptArgs[6] = state;
				attachInterrupt(6, class_isr6, CHANGE);
				break;
		#endif
		#ifdef CORE_INT7_PIN
			case CORE_INT7_PIN:
				interruptArgs[7] = state;
				attachInterrupt(7, class_isr7, CHANGE);
				break;
		#endif
		#ifdef CORE_INT8_PIN
			case CORE_INT8_PIN:
				interruptArgs[8] = state;
				attachInterrupt(8, class_isr8, CHANGE);
				break;
		#endif
		#ifdef CORE_INT9_PIN
			case CORE_INT9_PIN:
				interruptArgs[9] = state;
				attachInterrupt(9, class_isr9, CHANGE);
				break;
		#endif
		#ifdef CORE_INT10_PIN
			case CORE_INT10_PIN:
				interruptArgs[10] = state;
				attachInterrupt(10, class_isr10, CHANGE);
				break;
		#endif
		#ifdef CORE_INT11_PIN
			case CORE_INT11_PIN:
				interruptArgs[11] = state;
				attachInterrupt(11, class_isr11, CHANGE);
				break;
		#endif
		#ifdef CORE_INT12_PIN
			case CORE_INT12_PIN:
				interruptArgs[12] = state;
				attachInterrupt(12, class_isr12, CHANGE);
				break;
		#endif
		#ifdef CORE_INT13_PIN
			case CORE_INT13_PIN:
				interruptArgs[13] = state;
				attachInterrupt(13, class_isr13, CHANGE);
				break;
		#endif
		#ifdef CORE_INT14_PIN
			case CORE_INT14_PIN:
				interruptArgs[14] = state;
				attachInterrupt(14, class_isr14, CHANGE);
				break;
		#endif
		#ifdef CORE_INT15_PIN
			case CORE_INT15_PIN:
				interruptArgs[15] = state;
				attachInterrupt(15, class_isr15, CHANGE);
				break;
		#endif
		#ifdef CORE_INT16_PIN
			case CORE_INT16_PIN:
				interruptArgs[16] = state;
				attachInterrupt(16, class_isr16, CHANGE);
				break;
		#endif
		#ifdef CORE_INT17_PIN
			case CORE_INT17_PIN:
				interruptArgs[17] = state;
				attachInterrupt(17, class_isr17, CHANGE);
				break;
		#endif
		#ifdef CORE_INT18_PIN
			case CORE_INT18_PIN:
				interruptArgs[18] = state;
				attachInterrupt(18, class_isr18, CHANGE);
				break;
		#endif
		#ifdef CORE_INT19_PIN
			case CORE_INT19_PIN:
				interruptArgs[19] = state;
				attachInterrupt(19, class_isr19, CHANGE);
				break;
		#endif
		#ifdef CORE_INT20_PIN
			case CORE_INT20_PIN:
				interruptArgs[20] = state;
				attachInterrupt(20, class_isr20, CHANGE);
				break;
		#endif
		#ifdef CORE_INT21_PIN
			case CORE_INT21_PIN:
				interruptArgs[21] = state;
				attachInterrupt(21, class_isr21, CHANGE);
				break;
		#endif
		#ifdef CORE_INT22_PIN
			case CORE_INT22_PIN:
				interruptArgs[22] = state;
				attachInterrupt(22, class_isr22, CHANGE);
				break;
		#endif
		#ifdef CORE_INT23_PIN
			case CORE_INT23_PIN:
				interruptArgs[23] = state;
				attachInterrupt(23, class_isr23, CHANGE);
				break;
		#endif
		#ifdef CORE_INT24_PIN
			case CORE_INT24_PIN:
				interruptArgs[24] = state;
				attachInterrupt(24, class_isr24, CHANGE);
				break;
		#endif
		#ifdef CORE_INT25_PIN
			case CORE_INT25_PIN:
				interruptArgs[25] = state;
				attachInterrupt(25, class_isr25, CHANGE);
				break;
		#endif
		#ifdef CORE_INT26_PIN
			case CORE_INT26_PIN:
				interruptArgs[26] = state;
				attachInterrupt(26, class_isr26, CHANGE);
				break;
		#endif
		#ifdef CORE_INT27_PIN
			case CORE_INT27_PIN:
				interruptArgs[27] = state;
				attachInterrupt(27, class_isr27, CHANGE);
				break;
		#endif
		#ifdef CORE_INT28_PIN
			case CORE_INT28_PIN:
				interruptArgs[28] = state;
				attachInterrupt(28, class_isr28, CHANGE);
				break;
		#endif
		#ifdef CORE_INT29_PIN
			case CORE_INT29_PIN:
				interruptArgs[29] = state;
				attachInterrupt(29, class_isr29, CHANGE);
				break;
		#endif

		#ifdef CORE_INT30_PIN
			case CORE_INT30_PIN:
				interruptArgs[30] = state;
				attachInterrupt(30, class_isr30, CHANGE);
				break;
		#endif
		#ifdef CORE_INT31_PIN
			case CORE_INT31_PIN:
				interruptArgs[31] = state;
				attachInterrupt(31, class_isr31, CHANGE);
				break;
		#endif
		#ifdef CORE_INT32_PIN
			case CORE_INT32_PIN:
				interruptArgs[32] = state;
				attachInterrupt(32, class_isr32, CHANGE);
				break;
		#endif
		#ifdef CORE_INT33_PIN
			case CORE_INT33_PIN:
				interruptArgs[33] = state;
				attachInterrupt(33, class_isr33, CHANGE);
				break;
		#endif
		#ifdef CORE_INT34_PIN
			case CORE_INT34_PIN:
				interruptArgs[34] = state;
				attachInterrupt(34, class_isr34, CHANGE);
				break;
		#endif
		#ifdef CORE_INT35_PIN
			case CORE_INT35_PIN:
				interruptArgs[35] = state;
				attachInterrupt(35, class_isr35, CHANGE);
				break;
		#endif
		#ifdef CORE_INT36_PIN
			case CORE_INT36_PIN:
				interruptArgs[36] = state;
				attachInterrupt(36, class_isr36, CHANGE);
				break;
		#endif
		#ifdef CORE_INT37_PIN
			case CORE_INT37_PIN:
				interruptArgs[37] = state;
				attachInterrupt(37, class_isr37, CHANGE);
				break;
		#endif
		#ifdef CORE_INT38_PIN
			case CORE_INT38_PIN:
				interruptArgs[38] = state;
				attachInterrupt(38, class_isr38, CHANGE);
				break;
		#endif
		#ifdef CORE_INT39_PIN
			case CORE_INT39_PIN:
				interruptArgs[39] = state;
				attachInterrupt(39, class_isr39, CHANGE);
				break;
		#endif
		#ifdef CORE_INT40_PIN
			case CORE_INT40_PIN:
				interruptArgs[40] = state;
				attachInterrupt(40, class_isr40, CHANGE);
				break;
		#endif
		#ifdef CORE_INT41_PIN
			case CORE_INT41_PIN:
				interruptArgs[41] = state;
				attachInterrupt(41, class_isr41, CHANGE);
				break;
		#endif
		#ifdef CORE_INT42_PIN
			case CORE_INT42_PIN:
				interruptArgs[42] = state;
				attachInterrupt(42, class_isr42, CHANGE);
				break;
		#endif
		#ifdef CORE_INT43_PIN
			case CORE_INT43_PIN:
				interruptArgs[43] = state;
				attachInterrupt(43, class_isr43, CHANGE);
				break;
		#endif
		#ifdef CORE_INT44_PIN
			case CORE_INT44_PIN:
				interruptArgs[44] = state;
				attachInterrupt(44, class_isr44, CHANGE);
				break;
		#endif
		#ifdef CORE_INT45_PIN
			case CORE_INT45_PIN:
				interruptArgs[45] = state;
				attachInterrupt(45, class_isr45, CHANGE);
				break;
		#endif
		#ifdef CORE_INT46_PIN
			case CORE_INT46_PIN:
				interruptArgs[46] = state;
				attachInterrupt(46, class_isr46, CHANGE);
				break;
		#endif
		#ifdef CORE_INT47_PIN
			case CORE_INT47_PIN:
				interruptArgs[47] = state;
				attachInterrupt(47, class_isr47, CHANGE);
				break;
		#endif
		#ifdef CORE_INT48_PIN
			case CORE_INT48_PIN:
				interruptArgs[48] = state;
				attachInterrupt(48, class_isr48, CHANGE);
				break;
		#endif
		#ifdef CORE_INT49_PIN
			case CORE_INT49_PIN:
				interruptArgs[49] = state;
				attachInterrupt(49, class_isr49, CHANGE);
				break;
		#endif
		#ifdef CORE_INT50_PIN
			case CORE_INT50_PIN:
				interruptArgs[50] = state;
				attachInterrupt(50, class_isr50, CHANGE);
				break;
		#endif
		#ifdef CORE_INT51_PIN
			case CORE_INT51_PIN:
				interruptArgs[51] = state;
				attachInterrupt(51, class_isr51, CHANGE);
				break;
		#endif
		#ifdef CORE_INT52_PIN
			case CORE_INT52_PIN:
				interruptArgs[52] = state;
				attachInterrupt(52, class_isr52, CHANGE);
				break;
		#endif
		#ifdef CORE_INT53_PIN
			case CORE_INT53_PIN:
				interruptArgs[53] = state;
				attachInterrupt(53, class_isr53, CHANGE);
				break;
		#endif
		#ifdef CORE_INT54_PIN
			case CORE_INT54_PIN:
				interruptArgs[54] = state;
				attachInterrupt(54, class_isr54, CHANGE);
				break;
		#endif
		#ifdef CORE_INT55_PIN
			case CORE_INT55_PIN:
				interruptArgs[55] = state;
				attachInterrupt(55, class_isr55, CHANGE);
				break;
		#endif
		#ifdef CORE_INT56_PIN
			case CORE_INT56_PIN:
				interruptArgs[56] = state;
				attachInterrupt(56, class_isr56, CHANGE);
				break;
		#endif
		#ifdef CORE_INT57_PIN
			case CORE_INT57_PIN:
				interruptArgs[57] = state;
				attachInterrupt(57, class_isr57, CHANGE);
				break;
		#endif
		#ifdef CORE_INT58_PIN
			case CORE_INT58_PIN:
				interruptArgs[58] = state;
				attachInterrupt(58, class_isr58, CHANGE);
				break;
		#endif
		#ifdef CORE_INT59_PIN
			case CORE_INT59_PIN:
				interruptArgs[59] = state;
				attachInterrupt(59, class_isr59, CHANGE);
				break;
		#endif
			default:
				return 0;
		}
		return 1;
	}



#endif /* UTIL_H_ */
