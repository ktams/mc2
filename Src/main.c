/**
 ******************************************************************************
 * @file    main.c
 * @author  Andi
 ******************************************************************************
 */

/*
 * RB2, next generation model railroad controller software
 * Copyright (C) 2020 Tams Elektronik GmbH and Andreas Kretzer
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "rb2.h"
#include "backtrace.h"

// Let's start with some documentation and the copies of the license terms used
// within this software.
// ---------------------------------------------------------------------------------
// this is just documentation! This software as a whole is released under the GPLv2
// ---------------------------------------------------------------------------------

/**
 * @mainpage Next generation model railroad control
 *
 * This is a project initiated by
 *
 * Tams Elektronik GmbH, Hannover<br>
 * Andreas Kretzer Soft- und Hardwareentwicklung, Berlin
 *
 * It uses some freeware projects under different license models:
 *
 * FreeRTOS v10.2.1, now owned by Amazon Web Services, released under the @ref MIT <br>
 * lwIP 2.1.2, originallay developed by Adam Dunkels at the Swedish Institute of Computer Science, @ref mBSD_sics <br>
 * YAFFS, Aleph One Limited, @ref GPLv2 <br>
 * libogg 1.3.4, Xiph.Org Foundation, @ref mBSD_xiph <br>
 * opus 1.3.1, Xiph.Org ans others, @ref mBSD_opus
 *
 * The software as a whole is released under the @ref GPLv2
 */

/**
 * @page MIT MIT license
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * @page mBSD_sics Modified BSD license (SICS)
 *
 * Copyright (c) 2001-2004 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 * derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 */

/**
 * @page mBSD_xiph Modified BSD license (Xiph.org)
 *
 * Copyright (c) 2002, Xiph.org Foundation
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of the Xiph.org Foundation nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @page mBSD_opus Modified BSD license (Opus)
 *
 * Copyright 2001-2011 Xiph.Org, Skype Limited, Octasic,
 *                     Jean-Marc Valin, Timothy B. Terriberry,
 *                     CSIRO, Gregory Maxwell, Mark Borgerding,
 *                     Erik de Castro Lopo
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of Internet Society, IETF or IETF Trust, nor the
 * names of specific contributors, may be used to endorse or promote
 * products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Opus is subject to the royalty-free patent licenses which are
 * specified at:
 *
 * Xiph.Org Foundation:
 * https://datatracker.ietf.org/ipr/1524/
 *
 * Microsoft Corporation:
 * https://datatracker.ietf.org/ipr/1914/
 *
 * Broadcom Corporation:
 * https://datatracker.ietf.org/ipr/1526/
 */

/**
 * @page GPLv2 GPL version 2
 *
 * <h1>GNU GENERAL PUBLIC LICENSE</h1>
 *
 * Version 2, June 1991
 *
 * <pre>
 * Copyright (C) 1989, 1991 Free Software Foundation, Inc.
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 * 
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 * </pre>
 *
 * <h2>Preamble</h2>
 * 
 * The licenses for most software are designed to take away your freedom to share and change it.s
 * By contrast, the GNU General Public License is intended to guarantee your freedom to share and
 * change free software--to make sure the software is free for all its users. This General Public License
 * applies to most of the Free Software Foundation's software and to any other program whose authors
 * commit to using it. (Some other Free Software Foundation software is covered by the GNU Lesser
 * General Public License instead.) You can apply it to your programs, too.
 * 
 * When we speak of free software, we are referring to freedom, not price. Our General Public Licenses
 * are designed to make sure that you have the freedom to distribute copies of free software
 * (and charge for this service if you wish), that you receive source code or can get it if you
 * want it, that you can change the software or use pieces of it in new free programs; and that
 * you know you can do these things.
 * 
 * To protect your rights, we need to make restrictions that forbid anyone to deny you these rights
 * or to ask you to surrender the rights. These restrictions translate to certain responsibilities
 * for you if you distribute copies of the software, or if you modify it.
 * 
 * For example, if you distribute copies of such a program, whether gratis or for a fee, you must
 * give the recipients all the rights that you have. You must make sure that they, too, receive
 * or can get the source code. And you must show them these terms so they know their rights.
 * 
 * We protect your rights with two steps: (1) copyright the software, and (2) offer you this
 * license which gives you legal permission to copy, distribute and/or modify the software.
 * 
 * Also, for each author's protection and ours, we want to make certain that everyone understands
 * that there is no warranty for this free software. If the software is modified by someone else
 * and passed on, we want its recipients to know that what they have is not the original, so that
 * any problems introduced by others will not reflect on the original authors' reputations.
 * 
 * Finally, any free program is threatened constantly by software patents. We wish to avoid the
 * danger that redistributors of a free program will individually obtain patent licenses, in effect
 * making the program proprietary. To prevent this, we have made it clear that any patent must be
 * licensed for everyone's free use or not licensed at all.
 * 
 * The precise terms and conditions for copying, distribution and modification follow.
 *
 * <h2>TERMS AND CONDITIONS FOR COPYING, DISTRIBUTION AND MODIFICATION</h2>
 * 
 * <b>0.</b>
 * This License applies to any program or other work which contains a notice placed by the
 * copyright holder saying it may be distributed under the terms of this General Public License.
 * The "Program", below, refers to any such program or work, and a "work based on the Program"
 * means either the Program or any derivative work under copyright law: that is to say, a work
 * containing the Program or a portion of it, either verbatim or with modifications and/or translated
 * into another language. (Hereinafter, translation is included without limitation in the term
 * "modification".) Each licensee is addressed as "you".
 * 
 * Activities other than copying, distribution and modification are not covered by this License;
 * they are outside its scope. The act of running the Program is not restricted, and the output
 * from the Program is covered only if its contents constitute a work based on the Program
 * (independent of having been made by running the Program). Whether that is true depends on what
 * the Program does.
 * 
 * <b>1.</b>
 * You may copy and distribute verbatim copies of the Program's source code as you receive it,
 * in any medium, provided that you conspicuously and appropriately publish on each copy an
 * appropriate copyright notice and disclaimer of warranty; keep intact all the notices that refer
 * to this License and to the absence of any warranty; and give any other recipients of the Program
 * a copy of this License along with the Program.
 * 
 * You may charge a fee for the physical act of transferring a copy, and you may at your option offer
 * warranty protection in exchange for a fee.
 * 
 * <b>2.</b>
 * You may modify your copy or copies of the Program or any portion of it, thus forming a work
 * based on the Program, and copy and distribute such modifications or work under the terms of
 * Section 1 above, provided that you also meet all of these conditions:
 * 
 *   a) You must cause the modified files to carry prominent notices stating that you changed the
 *      files and the date of any change.
 *   b) You must cause any work that you distribute or publish, that in whole or in part contains
 *      or is derived from the Program or any part thereof, to be licensed as a whole at no charge to
 *      all third parties under the terms of this License.
 *   c) If the modified program normally reads commands interactively when run, you must cause it,
 *      when started running for such interactive use in the most ordinary way, to print or display
 *      an announcement including an appropriate copyright notice and a notice that there is no warranty
 *      (or else, saying that you provide a warranty) and that users may redistribute the program under
 *      these conditions, and telling the user how to view a copy of this License. (Exception: if the
 *      Program itself is interactive but does not normally print such an announcement, your work based
 *      on the Program is not required to print an announcement.)
 *
 * These requirements apply to the modified work as a whole. If identifiable sections of that work are
 * not derived from the Program, and can be reasonably considered independent and separate works in
 * themselves, then this License, and its terms, do not apply to those sections when you distribute them
 * as separate works. But when you distribute the same sections as part of a whole which is a work based
 * on the Program, the distribution of the whole must be on the terms of this License, whose permissions
 * for other licensees extend to the entire whole, and thus to each and every part regardless of who
 * wrote it.
 *
 * Thus, it is not the intent of this section to claim rights or contest your rights to work written
 * entirely by you; rather, the intent is to exercise the right to control the distribution of
 * derivative or collective works based on the Program.
 *
 * In addition, mere aggregation of another work not based on the Program with the Program (or with a
 * work based on the Program) on a volume of a storage or distribution medium does not bring the other
 * work under the scope of this License.
 *
 * <b>3.</b>
 * You may copy and distribute the Program (or a work based on it, under Section 2) in object code or
 * executable form under the terms of Sections 1 and 2 above provided that you also do one of the
 * following:
 *
 *   a) Accompany it with the complete corresponding machine-readable source code, which must be
 *      distributed under the terms of Sections 1 and 2 above on a medium customarily used for
 *      software interchange; or,
 *   b) Accompany it with a written offer, valid for at least three years, to give any third party,
 *      for a charge no more than your cost of physically performing source distribution, a complete
 *      machine-readable copy of the corresponding source code, to be distributed under the terms of
 *      Sections 1 and 2 above on a medium customarily used for software interchange; or,
 *   c) Accompany it with the information you received as to the offer to distribute corresponding
 *      source code. (This alternative is allowed only for noncommercial distribution and only if you
 *      received the program in object code or executable form with such an offer, in accord with
 *      Subsection b above.)
 *
 * The source code for a work means the preferred form of the work for making modifications to it.
 * For an executable work, complete source code means all the source code for all modules it contains,
 * plus any associated interface definition files, plus the scripts used to control compilation and
 * installation of the executable. However, as a special exception, the source code distributed need
 * not include anything that is normally distributed (in either source or binary form) with the major
 * components (compiler, kernel, and so on) of the operating system on which the executable runs,
 * unless that component itself accompanies the executable.
 *
 * If distribution of executable or object code is made by offering access to copy from a designated
 * place, then offering equivalent access to copy the source code from the same place counts as
 * distribution of the source code, even though third parties are not compelled to copy the source
 * along with the object code.
 *
 * <b>4.</b>
 * You may not copy, modify, sublicense, or distribute the Program except as expressly provided
 * under this License. Any attempt otherwise to copy, modify, sublicense or distribute the Program
 * is void, and will automatically terminate your rights under this License. However, parties who
 * have received copies, or rights, from you under this License will not have their licenses terminated
 * so long as such parties remain in full compliance.
 *
 * <b>5.</b>
 * You are not required to accept this License, since you have not signed it. However, nothing
 * else grants you permission to modify or distribute the Program or its derivative works. These
 * actions are prohibited by law if you do not accept this License. Therefore, by modifying or
 * distributing the Program (or any work based on the Program), you indicate your acceptance of
 * this License to do so, and all its terms and conditions for copying, distributing or modifying
 * the Program or works based on it.
 *
 * <b>6.</b>
 * Each time you redistribute the Program (or any work based on the Program), the recipient
 * automatically receives a license from the original licensor to copy, distribute or modify the
 * Program subject to these terms and conditions. You may not impose any further restrictions on
 * the recipients' exercise of the rights granted herein. You are not responsible for enforcing
 * compliance by third parties to this License.
 *
 * <b>7.</b>
 * If, as a consequence of a court judgment or allegation of patent infringement or for any
 * other reason (not limited to patent issues), conditions are imposed on you (whether by court
 * order, agreement or otherwise) that contradict the conditions of this License, they do not
 * excuse you from the conditions of this License. If you cannot distribute so as to satisfy
 * simultaneously your obligations under this License and any other pertinent obligations, then as
 * a consequence you may not distribute the Program at all. For example, if a patent license would
 * not permit royalty-free redistribution of the Program by all those who receive copies directly
 * or indirectly through you, then the only way you could satisfy both it and this License would
 * be to refrain entirely from distribution of the Program.
 *
 * If any portion of this section is held invalid or unenforceable under any particular
 * circumstance, the balance of the section is intended to apply and the section as a whole is
 * intended to apply in other circumstances.
 *
 * It is not the purpose of this section to induce you to infringe any patents or other property
 * right claims or to contest validity of any such claims; this section has the sole purpose of
 * protecting the integrity of the free software distribution system, which is implemented by
 * public license practices. Many people have made generous contributions to the wide range of
 * software distributed through that system in reliance on consistent application of that system;
 * it is up to the author/donor to decide if he or she is willing to distribute software through
 * any other system and a licensee cannot impose that choice.
 *
 * This section is intended to make thoroughly clear what is believed to be a consequence of the
 * rest of this License.
 *
 * <b>8.</b>
 * If the distribution and/or use of the Program is restricted in certain countries either by
 * patents or by copyrighted interfaces, the original copyright holder who places the Program
 * under this License may add an explicit geographical distribution limitation excluding those
 * countries, so that distribution is permitted only in or among countries not thus excluded.
 * In such case, this License incorporates the limitation as if written in the body of this License.
 *
 * <b>9.</b>
 * The Free Software Foundation may publish revised and/or new versions of the General Public
 * License from time to time. Such new versions will be similar in spirit to the present version,
 * but may differ in detail to address new problems or concerns.
 *
 * Each version is given a distinguishing version number. If the Program specifies a version number
 * of this License which applies to it and "any later version", you have the option of following
 * the terms and conditions either of that version or of any later version published by the Free
 * Software Foundation. If the Program does not specify a version number of this License, you may
 * choose any version ever published by the Free Software Foundation.
 *
 * <b>10.</b>
 * If you wish to incorporate parts of the Program into other free programs whose distribution
 * conditions are different, write to the author to ask for permission. For software which is
 * copyrighted by the Free Software Foundation, write to the Free Software Foundation; we sometimes
 * make exceptions for this. Our decision will be guided by the two goals of preserving the free
 * status of all derivatives of our free software and of promoting the sharing and reuse of
 * software generally.
 *
 * <h2>NO WARRANTY</h2>
 *
 * <b>11.</b>
 * BECAUSE THE PROGRAM IS LICENSED FREE OF CHARGE, THERE IS NO WARRANTY FOR THE PROGRAM, TO THE
 * EXTENT PERMITTED BY APPLICABLE LAW. EXCEPT WHEN OTHERWISE STATED IN WRITING THE COPYRIGHT HOLDERS
 * AND/OR OTHER PARTIES PROVIDE THE PROGRAM "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED
 * OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE. THE ENTIRE RISK AS TO THE QUALITY AND PERFORMANCE OF THE PROGRAM IS
 * WITH YOU. SHOULD THE PROGRAM PROVE DEFECTIVE, YOU ASSUME THE COST OF ALL NECESSARY SERVICING,
 * REPAIR OR CORRECTION.
 *
 * <b>12.</b>
 * IN NO EVENT UNLESS REQUIRED BY APPLICABLE LAW OR AGREED TO IN WRITING WILL ANY COPYRIGHT HOLDER,
 * OR ANY OTHER PARTY WHO MAY MODIFY AND/OR REDISTRIBUTE THE PROGRAM AS PERMITTED ABOVE, BE LIABLE TO
 * YOU FOR DAMAGES, INCLUDING ANY GENERAL, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES ARISING OUT OF
 * THE USE OR INABILITY TO USE THE PROGRAM (INCLUDING BUT NOT LIMITED TO LOSS OF DATA OR DATA BEING
 * RENDERED INACCURATE OR LOSSES SUSTAINED BY YOU OR THIRD PARTIES OR A FAILURE OF THE PROGRAM TO
 * OPERATE WITH ANY OTHER PROGRAMS), EVEN IF SUCH HOLDER OR OTHER PARTY HAS BEEN ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 *
 * <b>END OF TERMS AND CONDITIONS</b>
 *
 * <h2>How to Apply These Terms to Your New Programs</h2>
 *
 * If you develop a new program, and you want it to be of the greatest possible use to the public,
 * the best way to achieve this is to make it free software which everyone can redistribute and
 * change under these terms.
 *
 * To do so, attach the following notices to the program. It is safest to attach them to the start
 * of each source file to most effectively convey the exclusion of warranty; and each file should
 * have at least the "copyright" line and a pointer to where the full notice is found.
 *
 * <pre>
 * one line to give the program's name and an idea of what it does.
 * Copyright (C) yyyy  name of author
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 * </pre>
 *
 * Also add information on how to contact you by electronic and paper mail.
 *
 * If the program is interactive, make it output a short notice like this when it starts in an
 * interactive mode:
 *
 * <pre>
 * Gnomovision version 69, Copyright (C) year name of author
 * Gnomovision comes with ABSOLUTELY NO WARRANTY; for details
 * type `show w'.  This is free software, and you are welcome
 * to redistribute it under certain conditions; type `show c'
 * for details.
 * </pre>
 *
 * The hypothetical commands `show w' and `show c' should show the appropriate parts of the
 * General Public License. Of course, the commands you use may be called something other than
 * `show w' and `show c'; they could even be mouse-clicks or menu items--whatever suits your
 * program.
 *
 * You should also get your employer (if you work as a programmer) or your school, if any, to
 * sign a "copyright disclaimer" for the program, if necessary. Here is a sample; alter the names:
 *
 * <pre>
 * Yoyodyne, Inc., hereby disclaims all copyright
 * interest in the program `Gnomovision'
 * (which makes passes at compilers) written
 * by James Hacker.
 * 
 * signature of Ty Coon, 1 April 1989
 * Ty Coon, President of Vice
 * </pre>
 *
 * This General Public License does not permit incorporating your program into proprietary programs.
 * If your program is a subroutine library, you may consider it more useful to permit linking proprietary
 * applications with the library. If this is what you want to do, use the GNU Lesser General Public
 * License instead of this License.
 */

const struct hwinfo *hwinfo = (const struct hwinfo *) 0x0801FFE0;

#define DEVELOPMENT		1		// if defined to '1', the boot loader doesn't mourne about undefined firmware or hardware type
#define SOURCE_BASE		"RB2-Main"		// for assertions: find this string to cut the leading portion of the filename (the individual local part from compilation)

#if DEVELOPMENT == 1
static const char insecure[] = "InSeCurEInSeCurE";		// a marker for unchecked HEX files
#endif

extern uint32_t _end;
struct runtime rt;
const flexval fvNULL = {.i32 = 0};		// convinience variable for unused callback priv parameter

typedef struct __attribute__((packed)) ContextStateFrame {
  uint32_t r0;
  uint32_t r1;
  uint32_t r2;
  uint32_t r3;
  uint32_t r12;
  uint32_t lr;
  uint32_t return_address;
  uint32_t xpsr;
} sContextStateFrame;

static void dump_backtrace(const backtrace_t *backtrace, int count)
{
	for (int i = 0; i < count; ++i)
		printf("\t%p - %s @ %p\n", backtrace[i].function, backtrace[i].name, backtrace[i].address);
}

void killThread (uint32_t address, uint32_t cfsr)
{
	backtrace_t backtrace[20];
	int size;

	fprintf (stderr, "%s() from %s\n", __func__, pcTaskGetName(NULL));
	fprintf (stderr, "\t@ 0x%08lx\n", address);
	// MemManage errors:
	if (cfsr & SCB_CFSR_MMARVALID_Msk)		fprintf (stderr, "\tMemManage FAULT ADDRESS: %p\n", (void *) SCB->MMFAR);
	if (cfsr & SCB_CFSR_MLSPERR_Msk)		fprintf (stderr, "\tMemManage during floating-point lazy state preservation\n");
	if (cfsr & SCB_CFSR_MSTKERR_Msk)		fprintf (stderr, "\tMemManage Exception entry caused stack access violation\n");
	if (cfsr & SCB_CFSR_MUNSTKERR_Msk)		fprintf (stderr, "\tMemManage Exception return caused stack access violation\n");
	if (cfsr & SCB_CFSR_DACCVIOL_Msk)		fprintf (stderr, "\tMemManage DATA access violation\n");
	if (cfsr & SCB_CFSR_MLSPERR_Msk)		fprintf (stderr, "\tMemManage INSTRUCTION access violation\n");

	// Bus errors:
	if (cfsr & SCB_CFSR_BFARVALID_Msk)		fprintf (stderr, "\tBusError FAULT ADDRESS: %p\n", (void *) SCB->BFAR);
	if (cfsr & SCB_CFSR_LSPERR_Msk)			fprintf (stderr, "\tBusError during floating-point lazy state preservation\n");
	if (cfsr & SCB_CFSR_STKERR_Msk)			fprintf (stderr, "\tBusError Exception entry caused stack access violation\n");
	if (cfsr & SCB_CFSR_UNSTKERR_Msk)		fprintf (stderr, "\tBusError Exception return caused stack access violation\n");
	if (cfsr & SCB_CFSR_IMPRECISERR_Msk)	fprintf (stderr, "\tBusError Return address is IMPRECISE\n");
	if (cfsr & SCB_CFSR_PRECISERR_Msk)		fprintf (stderr, "\tBusError Return address is PRECISE\n");
	if (cfsr & SCB_CFSR_IBUSERR_Msk)		fprintf (stderr, "\tBusError INSTRUCTION bus prefetch\n");

	// UsageFault error:
	if (cfsr & SCB_CFSR_DIVBYZERO_Msk)		fprintf (stderr, "\tUsageFault DIVISION BY ZERO\n");
	if (cfsr & SCB_CFSR_UNALIGNED_Msk)		fprintf (stderr, "\tUsageFault UNALIGNED access\n");
	if (cfsr & SCB_CFSR_NOCP_Msk)			fprintf (stderr, "\tUsageFault NO CO-Processor\n");
	if (cfsr & SCB_CFSR_INVPC_Msk)			fprintf (stderr, "\tUsageFault INVALID PC LOAD usage\n");
	if (cfsr & SCB_CFSR_INVSTATE_Msk)		fprintf (stderr, "\tUsageFault ILLEGAL USE OF EPSR\n");
	if (cfsr & SCB_CFSR_UNDEFINSTR_Msk)		fprintf (stderr, "\tUsageFault UNDEFINED INSTRUCTION\n");

	printf ("\t----- STACK BACKTRACE -----\n");		// bad luck: this cannot print the real stack ... (maybe we later find a way)
	size = backtrace_unwind(backtrace, DIM(backtrace));
	dump_backtrace(backtrace, size);

	vTaskDelay(1000);
	SCB->AIRCR = (0x05FA << 16) | 0x1 << 2;			// do a RESET
	while (1) ;										// not reached
}

__attribute__((optimize("O0")))
void faultHandler_c (sContextStateFrame *frame)
{
	if ((frame->xpsr & 0xFF) != 0) {		// in exception handler we cannot return and get back to normal operation - so just reset!
		SCB->AIRCR = (0x05FA << 16) | 0x1 << 2;			// do a RESET
		while (1) ;										// not reached
	}

	// try to recover and kill calling thread
	frame->r0 = frame->return_address;		// first parameter to DEBUG function (address where error occured)
	frame->r1 = SCB->CFSR;					// second parameter: the status register

	SCB->CFSR = SCB->CFSR;		// all set flags are written as '1' to reset them
	frame->return_address = (uint32_t) killThread;
	frame->lr = 0xdeadbeef;		// just in case the called function does something wrong
	frame->xpsr = 1 << 24;		// clear all flags in PSR and leave only the "thumb instruction interworking" set
}

void vAssertReasonCalled(const char * pcReason, const char * pcFile, unsigned long ulLine)
{
	fprintf (stderr, "%s() on line %ld: %s\n", pcFile, ulLine, pcReason);
}

void vAssertCalled(const char * pcFile, unsigned long ulLine, const char * pcFunc, const char *failedexpr)
{
	backtrace_t backtrace[20];
	TaskStatus_t ts;
	char *s;
	int size;

	if ((s = strstr(pcFile, SOURCE_BASE)) != NULL) pcFile = s;
	log_error ("%s ASSERTION \"%s\" in %s() on line %lu\n", pcFile, failedexpr, pcFunc, ulLine);
	vTaskGetInfo(xTaskGetCurrentTaskHandle(), &ts, pdTRUE, pdFALSE);
	printf ("\ttask '%s'\n", ts.pcTaskName);
	printf ("\tcurrent priority %ld\n", ts.uxCurrentPriority);
	printf ("\tbase priority %ld\n", ts.uxBasePriority);
	printf ("\tlowest stack space %d\n", ts.usStackHighWaterMark);
	printf ("\tRETURN address %p\n", __builtin_return_address(0));
	printf ("\tFRAME POINTER %p\n", __builtin_frame_address(0));

	printf ("\t----- STACK BACKTRACE -----\n");
	size = backtrace_unwind(backtrace, DIM(backtrace));
	dump_backtrace(backtrace, size);

	vTaskDelay(1000);
	if (!strcmp (ts.pcTaskName, "REBOOT")) {	// the REBOOT task had an assertion - make a direkt reset here
		NVIC_SystemReset();						// does not return!
	} else {
		reboot();
	}
	vTaskDelete(NULL);
}

void vApplicationStackOverflowHook (TaskHandle_t t, char *name)
{
	fprintf (stderr, "STACK OVERFLOW in Task '%s'...\n", name);
	vTaskDelete(t);
}

/**
 * Callback-Funktion des FreeRTOS Tick-Timers (ms-Interrupt).
 * Die Funktion wird tatsächlich im Kontext des Interrupts
 * aufgerufen.
 */
void vApplicationTickHook (void)
{
	seg_timer();
	key_scan();
	ts_handler();
	if ((WWDG1->CR & WWDG_CR_T_Msk) <= (WWDG1->CFR & WWDG_CFR_W_Msk)) WWDG1->CR = WWDG_CR_T_Msk;
}

int main(void)
{
	HeapRegion_t heap[4], *h;
	uint32_t ep;

#if DEVELOPMENT == 1
	log_enable(LOG_DEBUG);
#endif

	hw_setup();

	memset (&rt, 0, sizeof(rt));
	h = heap;
	// D1-SRAM (512K) via AXI is located @ 0x24000000 and contains all static allocated variables (so size is calculated dynamically).
	// The initial Stack (IRQ-Stack?) is located down from the end of this RAM. We reserve INITIAL_STACK_SIZE (2k) for this stack.
	ep = (uint32_t) &_end;
	ep += 0x100;
	ep &= ~0xFF;	// align heap base to an address that lies on a 0x100 boundary
	h->pucStartAddress = (uint8_t *) ep;
	h->xSizeInBytes = ((uint8_t *) (D1_AXISRAM_BASE + D1_AXISRAM_SIZE)) - INITIAL_STACK_SIZE - h->pucStartAddress;
	rt.totalHeap += h->xSizeInBytes;

	h++;
	// SRAM1 + SRAM2 + SRAM3 in D2-Domain (288k) via AHB is located @ 0x30000000
	h->pucStartAddress = (uint8_t *) (D2_AHBSRAM_BASE + 0x1000);	// 4k are reserved for the ethernet buffer descriptors
	h->xSizeInBytes = (284 * 1024);									// 288k D2-SRAM minus the 4k reserved space
	rt.totalHeap += h->xSizeInBytes;

	h++;
	// external SDRAM (8M) is located @ 0x60000000
	h->pucStartAddress = (uint8_t *) SDRAM_BASE;
	h->xSizeInBytes = SDRAM_SIZE;
	rt.totalHeap += h->xSizeInBytes;

	h++;
	h->pucStartAddress = NULL;
	h->xSizeInBytes = 0;
	vPortDefineHeapRegions(heap);

	printf ("_end = %p\n", &_end);
	for (h = heap; h->pucStartAddress != NULL; h++) {
		printf ("%p: %d bytes\n", h->pucStartAddress, h->xSizeInBytes);
	}

	printf ("--- end of memory regions ---\n");

	printf ("\r\nRB2 Manufacturer %d, HW %02x, serial %d\n", hwinfo->manufacturer, hwinfo->HW, hwinfo->serial);
	if (*hwinfo->proddate != 0xFF) printf ("Prod-Date %s\n", hwinfo->proddate);

	__enable_irq();		// from here on we can use interrupts

	xTaskCreate(vInit, "init", 1024, NULL, 1, NULL);
    vTaskStartScheduler();

#if DEVELOPMENT == 1
	printf ("%s\n", insecure);		// if DEVELOPMENT is defined or not - this is never executed (but the compiler can't know that)!
#endif
}
