/*
 * Copyright (c) 2011-2012, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mfd/pm8xxx/pm8xxx-adc.h>

#include <linux/pcb_version.h>

#define KELVINMIL_DEGMIL	273160

/* Units for temperature below (on x axis) is in 0.1DegC as
   required by the battery driver. Note the resolution used
   here to compute the table was done for DegC to milli-volts.
   In consideration to limit the size of the table for the given
   temperature range below, the result is linearly interpolated
   and provided to the battery driver in the units desired for
   their framework which is 0.1DegC. True resolution of 0.1DegC
   will result in the below table size to increase by 10 times */
   
/* OPPO 2012-08-10 chendx Modify begin for BTM */
#ifndef CONFIG_VENDOR_EDIT
static const struct pm8xxx_adc_map_pt adcmap_btm_threshold[] = {
	{-300,	1642},
	{-200,	1544},
	{-100,	1414},
	{0,	1260},
	{10,	1244},
	{20,	1228},
	{30,	1212},
	{40,	1195},
	{50,	1179},
	{60,	1162},
	{70,	1146},
	{80,	1129},
	{90,	1113},
	{100,	1097},
	{110,	1080},
	{120,	1064},
	{130,	1048},
	{140,	1032},
	{150,	1016},
	{160,	1000},
	{170,	985},
	{180,	969},
	{190,	954},
	{200,	939},
	{210,	924},
	{220,	909},
	{230,	894},
	{240,	880},
	{250,	866},
	{260,	852},
	{270,	838},
	{280,	824},
	{290,	811},
	{300,	798},
	{310,	785},
	{320,	773},
	{330,	760},
	{340,	748},
	{350,	736},
	{360,	725},
	{370,	713},
	{380,	702},
	{390,	691},
	{400,	681},
	{410,	670},
	{420,	660},
	{430,	650},
	{440,	640},
	{450,	631},
	{460,	622},
	{470,	613},
	{480,	604},
	{490,	595},
	{500,	587},
	{510,	579},
	{520,	571},
	{530,	563},
	{540,	556},
	{550,	548},
	{560,	541},
	{570,	534},
	{580,	527},
	{590,	521},
	{600,	514},
	{610,	508},
	{620,	502},
	{630,	496},
	{640,	490},
	{650,	485},
	{660,	281},
	{670,	274},
	{680,	267},
	{690,	260},
	{700,	254},
	{710,	247},
	{720,	241},
	{730,	235},
	{740,	229},
	{750,	224},
	{760,	218},
	{770,	213},
	{780,	208},
	{790,	203}
};
#else
/*OPPO*/
static const struct pm8xxx_adc_map_pt adcmap_btm_threshold[] = {
	{-350, 1473}, 
	{-340, 1460}, 
	{-330, 1447}, 
	{-320, 1434}, 
	{-310, 1421}, 
	{-300, 1408}, 
	{-290, 1395}, 
	{-280, 1381}, 
	{-270, 1368}, 
	{-260, 1354}, 
	{-250, 1341}, 
	{-240, 1327}, 
	{-230, 1313}, 
	{-220, 1300}, 
	{-210, 1286}, 
	{-200, 1273}, 
	{-190, 1259}, 
	{-180, 1246}, 
	{-170, 1232}, 
	{-160, 1219}, 
	{-150, 1206}, 
	{-140, 1193}, 
	{-130, 1180}, 
	{-120, 1173}, 
	{-110, 1167}, 
	{-100, 1155}, 
	{-90 , 1142}, 
	{-80 , 1130}, 
	{-70 , 1118}, 
	{-60 , 1106}, 
	{-50 , 1094}, 
	{-40 , 1083}, 
	{-30 , 1072}, 
	{-20 , 1061}, 
	{-10 , 1050}, 
	{0	, 1039}, 
	{10	, 1029}, 
	{20	, 1019}, 
	{30	, 1009}, 
	{40	, 999 }, 
	{50	, 990 }, 
	{60	, 981 }, 
	{70	, 972 }, 
	{80	, 963 }, 
	{90	, 954 }, 
	{100, 946 }, 
	{110, 938 }, 
	{120, 930 }, 
	{130, 922 }, 
	{140, 915 }, 
	{150, 908 }, 
	{160, 901 }, 
	{170, 894 }, 
	{180, 887 }, 
	{190, 881 }, 
	{200, 875 }, 
	{210, 869 }, 
	{220, 863 }, 
	{230, 857 }, 
	{240, 852 }, 
	{250, 847 }, 
	{260, 841 }, 
	{270, 836 }, 
	{280, 832 }, 
	{290, 827 }, 
	{300, 822 }, 
	{310, 818 }, 
	{320, 814 }, 
	{330, 810 }, 
	{340, 806 }, 
	{350, 802 }, 
	{360, 798 }, 
	{370, 794 }, 
	{380, 791 }, 
	{390, 788 }, 
	{400, 784 }, 
	{410, 781 }, 
	{420, 778 }, 
	{430, 775 }, 
	{440, 772 }, 
	{450, 770 }, 
	{460, 767 }, 
	{470, 764 }, 
	{480, 762 }, 
	{490, 759 }, 
	{500, 757 }, 
	{510, 755 }, 
	{520, 746 }, 
	{530, 745 }, 
	{540, 743 }, 
	{550, 741 }, 
	{560, 739 }, 
	{570, 737 }, 
	{580, 736 }, 
	{590, 734 }, 
	{600, 733 }, 
	{610, 731 }, 
	{620, 730 }, 
	{630, 729 }, 
	{640, 727 }, 
	{650, 726 }, 
	{660, 725 }, 
	{670, 724 }, 
	{680, 722 }, 
	{690, 721 }, 
	{700, 720 }, 
	{710, 719 }, 
	{720, 718 }, 
	{730, 717 }, 
	{740, 716 }, 
	{750, 715 }  
};

static const struct pm8xxx_adc_map_pt adcmap_btm_threshold2[] = {
	{-350, 1563}, 
	{-340, 1553}, 
	{-330, 1541}, 
	{-320, 1529}, 
	{-310, 1518}, 
	{-300, 1506}, 
	{-290, 1494}, 
	{-280, 1481}, 
	{-270, 1468}, 
	{-260, 1455}, 
	{-250, 1441}, 
	{-240, 1427}, 
	{-230, 1413}, 
	{-220, 1399}, 
	{-210, 1385}, 
	{-200, 1370}, 
	{-190, 1355}, 
	{-180, 1340}, 
	{-170, 1325}, 
	{-160, 1310}, 
	{-150, 1294}, 
	{-140, 1279}, 
	{-130, 1263}, 
	{-120, 1254}, 
	{-110, 1247}, 
	{-100, 1231}, 
	{-90 , 1215}, 
	{-80 , 1199}, 
	{-70 , 1183}, 
	{-60 , 1167}, 
	{-50 , 1151}, 
	{-40 , 1135}, 
	{-30 , 1119}, 
	{-20 , 1103}, 
	{-10 , 1087}, 
	{0	, 1071}, 
	{10	, 1055}, 
	{20	, 1040}, 
	{30	, 1024}, 
	{40	, 1009 }, 
	{50	, 993 }, 
	{60	, 979 }, 
	{70	, 964 }, 
	{80	, 949 }, 
	{90	, 935 }, 
	{100, 920 }, 
	{110, 906 }, 
	{120, 892 }, 
	{130, 879 }, 
	{140, 865 }, 
	{150, 852 }, 
	{160, 839 }, 
	{170, 826 }, 
	{180, 814 }, 
	{190, 801 }, 
	{200, 789 }, 
	{210, 777 }, 
	{220, 766 }, 
	{230, 755 }, 
	{240, 743 }, 
	{250, 733 }, 
	{260, 722 }, 
	{270, 712 }, 
	{280, 701 }, 
	{290, 692 }, 
	{300, 682 }, 
	{310, 672 }, 
	{320, 663 }, 
	{330, 654 }, 
	{340, 645 }, 
	{350, 637 }, 
	{360, 629 }, 
	{370, 621 }, 
	{380, 613 }, 
	{390, 605 }, 
	{400, 598 }, 
	{410, 590 }, 
	{420, 583 }, 
	{430, 576 }, 
	{440, 570 }, 
	{450, 563 }, 
	{460, 557 }, 
	{470, 551 }, 
	{480, 545 }, 
	{490, 539 }, 
	{500, 533 }, 
	{510, 528 }, 
	{520, 522 }, 
	{530, 517 }, 
	{540, 512 }, 
	{550, 507 }, 
	{560, 503 }, 
	{570, 498 }, 
	{580, 494 }, 
	{590, 489 }, 
	{600, 485 }, 
	{610, 481 }, 
	{620, 477 }, 
	{630, 473 }, 
	{640, 470 }, 
	{650, 466 }, 
	{660, 463 }, 
	{670, 459 }, 
	{680, 456 }, 
	{690, 453 }, 
	{700, 450 }, 
	{710, 447 }, 
	{720, 444 }, 
	{730, 441 }, 
	{740, 438 }, 
	{750, 436 }  
};	

#endif
/* OPPO 2012-08-10 chendx Modify end */
static const struct pm8xxx_adc_map_pt adcmap_pa_therm[] = {
	{1731,	-30},
	{1726,	-29},
	{1721,	-28},
	{1715,	-27},
	{1710,	-26},
	{1703,	-25},
	{1697,	-24},
	{1690,	-23},
	{1683,	-22},
	{1675,	-21},
	{1667,	-20},
	{1659,	-19},
	{1650,	-18},
	{1641,	-17},
	{1632,	-16},
	{1622,	-15},
	{1611,	-14},
	{1600,	-13},
	{1589,	-12},
	{1577,	-11},
	{1565,	-10},
	{1552,	-9},
	{1539,	-8},
	{1525,	-7},
	{1511,	-6},
	{1496,	-5},
	{1481,	-4},
	{1465,	-3},
	{1449,	-2},
	{1432,	-1},
	{1415,	0},
	{1398,	1},
	{1380,	2},
	{1362,	3},
	{1343,	4},
	{1324,	5},
	{1305,	6},
	{1285,	7},
	{1265,	8},
	{1245,	9},
	{1224,	10},
	{1203,	11},
	{1182,	12},
	{1161,	13},
	{1139,	14},
	{1118,	15},
	{1096,	16},
	{1074,	17},
	{1052,	18},
	{1030,	19},
	{1008,	20},
	{986,	21},
	{964,	22},
	{943,	23},
	{921,	24},
	{899,	25},
	{878,	26},
	{857,	27},
	{836,	28},
	{815,	29},
	{794,	30},
	{774,	31},
	{754,	32},
	{734,	33},
	{714,	34},
	{695,	35},
	{676,	36},
	{657,	37},
	{639,	38},
	{621,	39},
	{604,	40},
	{586,	41},
	{570,	42},
	{553,	43},
	{537,	44},
	{521,	45},
	{506,	46},
	{491,	47},
	{476,	48},
	{462,	49},
	{448,	50},
	{435,	51},
	{421,	52},
	{409,	53},
	{396,	54},
	{384,	55},
	{372,	56},
	{361,	57},
	{350,	58},
	{339,	59},
	{329,	60},
	{318,	61},
	{309,	62},
	{299,	63},
	{290,	64},
	{281,	65},
	{272,	66},
	{264,	67},
	{256,	68},
	{248,	69},
	{240,	70},
	{233,	71},
	{226,	72},
	{219,	73},
	{212,	74},
	{206,	75},
	{199,	76},
	{193,	77},
	{187,	78},
	{182,	79},
	{176,	80},
	{171,	81},
	{166,	82},
	{161,	83},
	{156,	84},
	{151,	85},
	{147,	86},
	{142,	87},
	{138,	88},
	{134,	89},
	{130,	90},
	{126,	91},
	{122,	92},
	{119,	93},
	{115,	94},
	{112,	95},
	{109,	96},
	{106,	97},
	{103,	98},
	{100,	99},
	{97,	100},
	{94,	101},
	{91,	102},
	{89,	103},
	{86,	104},
	{84,	105},
	{82,	106},
	{79,	107},
	{77,	108},
	{75,	109},
	{73,	110},
	{71,	111},
	{69,	112},
	{67,	113},
	{65,	114},
	{64,	115},
	{62,	116},
	{60,	117},
	{59,	118},
	{57,	119},
	{56,	120},
	{54,	121},
	{53,	122},
	{51,	123},
	{50,	124},
	{49,	125}
};

static const struct pm8xxx_adc_map_pt adcmap_ntcg_104ef_104fb[] = {
	{696483,	-40960},
	{649148,	-39936},
	{605368,	-38912},
	{564809,	-37888},
	{527215,	-36864},
	{492322,	-35840},
	{460007,	-34816},
	{429982,	-33792},
	{402099,	-32768},
	{376192,	-31744},
	{352075,	-30720},
	{329714,	-29696},
	{308876,	-28672},
	{289480,	-27648},
	{271417,	-26624},
	{254574,	-25600},
	{238903,	-24576},
	{224276,	-23552},
	{210631,	-22528},
	{197896,	-21504},
	{186007,	-20480},
	{174899,	-19456},
	{164521,	-18432},
	{154818,	-17408},
	{145744,	-16384},
	{137265,	-15360},
	{129307,	-14336},
	{121866,	-13312},
	{114896,	-12288},
	{108365,	-11264},
	{102252,	-10240},
	{96499,		-9216},
	{91111,		-8192},
	{86055,		-7168},
	{81308,		-6144},
	{76857,		-5120},
	{72660,		-4096},
	{68722,		-3072},
	{65020,		-2048},
	{61538,		-1024},
	{58261,		0},
	{55177,		1024},
	{52274,		2048},
	{49538,		3072},
	{46962,		4096},
	{44531,		5120},
	{42243,		6144},
	{40083,		7168},
	{38045,		8192},
	{36122,		9216},
	{34308,		10240},
	{32592,		11264},
	{30972,		12288},
	{29442,		13312},
	{27995,		14336},
	{26624,		15360},
	{25333,		16384},
	{24109,		17408},
	{22951,		18432},
	{21854,		19456},
	{20807,		20480},
	{19831,		21504},
	{18899,		22528},
	{18016,		23552},
	{17178,		24576},
	{16384,		25600},
	{15631,		26624},
	{14916,		27648},
	{14237,		28672},
	{13593,		29696},
	{12976,		30720},
	{12400,		31744},
	{11848,		32768},
	{11324,		33792},
	{10825,		34816},
	{10354,		35840},
	{9900,		36864},
	{9471,		37888},
	{9062,		38912},
	{8674,		39936},
	{8306,		40960},
	{7951,		41984},
	{7616,		43008},
	{7296,		44032},
	{6991,		45056},
	{6701,		46080},
	{6424,		47104},
	{6160,		48128},
	{5908,		49152},
	{5667,		50176},
	{5439,		51200},
	{5219,		52224},
	{5010,		53248},
	{4810,		54272},
	{4619,		55296},
	{4440,		56320},
	{4263,		57344},
	{4097,		58368},
	{3938,		59392},
	{3785,		60416},
	{3637,		61440},
	{3501,		62464},
	{3368,		63488},
	{3240,		64512},
	{3118,		65536},
	{2998,		66560},
	{2889,		67584},
	{2782,		68608},
	{2680,		69632},
	{2581,		70656},
	{2490,		71680},
	{2397,		72704},
	{2310,		73728},
	{2227,		74752},
	{2147,		75776},
	{2064,		76800},
	{1998,		77824},
	{1927,		78848},
	{1860,		79872},
	{1795,		80896},
	{1736,		81920},
	{1673,		82944},
	{1615,		83968},
	{1560,		84992},
	{1507,		86016},
	{1456,		87040},
	{1407,		88064},
	{1360,		89088},
	{1314,		90112},
	{1271,		91136},
	{1228,		92160},
	{1189,		93184},
	{1150,		94208},
	{1112,		95232},
	{1076,		96256},
	{1042,		97280},
	{1008,		98304},
	{976,		99328},
	{945,		100352},
	{915,		101376},
	{886,		102400},
	{859,		103424},
	{832,		104448},
	{807,		105472},
	{782,		106496},
	{756,		107520},
	{735,		108544},
	{712,		109568},
	{691,		110592},
	{670,		111616},
	{650,		112640},
	{631,		113664},
	{612,		114688},
	{594,		115712},
	{577,		116736},
	{560,		117760},
	{544,		118784},
	{528,		119808},
	{513,		120832},
	{498,		121856},
	{483,		122880},
	{470,		123904},
	{457,		124928},
	{444,		125952},
	{431,		126976},
	{419,		128000}
};

static int32_t pm8xxx_adc_map_linear(const struct pm8xxx_adc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if ((pts == NULL) || (output == NULL))
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].x < pts[1].x)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].x < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) &&
				(pts[i].x > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0)
		*output = pts[0].y;
	else if (i == tablesize)
		*output = pts[tablesize-1].y;
	else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].y - pts[i-1].y)*
			(input - pts[i-1].x))/
			(pts[i].x - pts[i-1].x))+
			pts[i-1].y);
	}

	return 0;
}

static int32_t pm8xxx_adc_map_batt_therm(const struct pm8xxx_adc_map_pt *pts,
		uint32_t tablesize, int32_t input, int64_t *output)
{
	bool descending = 1;
	uint32_t i = 0;

	if ((pts == NULL) || (output == NULL))
		return -EINVAL;

	/* Check if table is descending or ascending */
	if (tablesize > 1) {
		if (pts[0].y < pts[1].y)
			descending = 0;
	}

	while (i < tablesize) {
		if ((descending == 1) && (pts[i].y < input)) {
			/* table entry is less than measured
				value and table is descending, stop */
			break;
		} else if ((descending == 0) && (pts[i].y > input)) {
			/* table entry is greater than measured
				value and table is ascending, stop */
			break;
		} else {
			i++;
		}
	}

	if (i == 0) {
		*output = pts[0].x;
	} else if (i == tablesize) {
		*output = pts[tablesize-1].x;
	} else {
		/* result is between search_index and search_index-1 */
		/* interpolate linearly */
		*output = (((int32_t) ((pts[i].x - pts[i-1].x)*
			(input - pts[i-1].y))/
			(pts[i].y - pts[i-1].y))+
			pts[i-1].x);
	}

	return 0;
}

int32_t pm8xxx_adc_scale_default(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	bool negative_rawfromoffset = 0, negative_offset = 0;
	int64_t scale_voltage = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	scale_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;
	if (scale_voltage < 0) {
		negative_offset = 1;
		scale_voltage = -scale_voltage;
	}
	do_div(scale_voltage,
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dy);
	if (negative_offset)
		scale_voltage = -scale_voltage;
	scale_voltage += chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;

	if (scale_voltage < 0) {
		if (adc_properties->bipolar) {
			scale_voltage = -scale_voltage;
			negative_rawfromoffset = 1;
		} else {
			scale_voltage = 0;
		}
	}

	adc_chan_result->measurement = scale_voltage *
				chan_properties->offset_gain_denominator;

	/* do_div only perform positive integer division! */
	do_div(adc_chan_result->measurement,
				chan_properties->offset_gain_numerator);

	if (negative_rawfromoffset)
		adc_chan_result->measurement = -adc_chan_result->measurement;

	/* Note: adc_chan_result->measurement is in the unit of
	 * adc_properties.adc_reference. For generic channel processing,
	 * channel measurement is a scale/ratio relative to the adc
	 * reference input */
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_default);

static int64_t pm8xxx_adc_scale_ratiometric_calib(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties)
{
	int64_t adc_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties)
		return -EINVAL;

	adc_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd)
		* adc_properties->adc_vdd_reference;
	if (adc_voltage < 0) {
		negative_offset = 1;
		adc_voltage = -adc_voltage;
	}
	do_div(adc_voltage,
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy);
	if (negative_offset)
		adc_voltage = -adc_voltage;

	return adc_voltage;
}

int32_t pm8xxx_adc_scale_batt_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t bat_voltage = 0;

	bat_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	if(get_pcb_version() == PCB_VERSION_PVT3_TD) {
		return pm8xxx_adc_map_batt_therm(
			adcmap_btm_threshold2,
			ARRAY_SIZE(adcmap_btm_threshold2),
			bat_voltage,
			&adc_chan_result->physical);
	}else {
		return pm8xxx_adc_map_batt_therm(
			adcmap_btm_threshold,
			ARRAY_SIZE(adcmap_btm_threshold),
			bat_voltage,
			&adc_chan_result->physical);
	}
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_batt_therm);

int32_t pm8xxx_adc_scale_pa_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t pa_voltage = 0;

	pa_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);

	return pm8xxx_adc_map_linear(
			adcmap_pa_therm,
			ARRAY_SIZE(adcmap_pa_therm),
			pa_voltage,
			&adc_chan_result->physical);
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_pa_therm);

int32_t pm8xxx_adc_scale_batt_id(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t batt_id_voltage = 0;

	batt_id_voltage = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	adc_chan_result->physical = batt_id_voltage;
	adc_chan_result->physical = adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_batt_id);

int32_t pm8xxx_adc_scale_pmic_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t pmic_voltage = 0;
	bool negative_offset = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	pmic_voltage = (adc_code -
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].adc_gnd)
		* chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;
	if (pmic_voltage < 0) {
		negative_offset = 1;
		pmic_voltage = -pmic_voltage;
	}
	do_div(pmic_voltage,
		chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dy);
	if (negative_offset)
		pmic_voltage = -pmic_voltage;
	pmic_voltage += chan_properties->adc_graph[ADC_CALIB_ABSOLUTE].dx;

	if (pmic_voltage > 0) {
		/* 2mV/K */
		adc_chan_result->measurement = pmic_voltage*
			chan_properties->offset_gain_denominator;

		do_div(adc_chan_result->measurement,
			chan_properties->offset_gain_numerator * 2);
	} else {
		adc_chan_result->measurement = 0;
	}
	/* Change to .001 deg C */
	adc_chan_result->measurement -= KELVINMIL_DEGMIL;
	adc_chan_result->physical = (int32_t)adc_chan_result->measurement;

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_scale_pmic_therm);

/* Scales the ADC code to 0.001 degrees C using the map
 * table for the XO thermistor.
 */
int32_t pm8xxx_adc_tdkntcg_therm(int32_t adc_code,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties,
		struct pm8xxx_adc_chan_result *adc_chan_result)
{
	int64_t xo_thm = 0;

	if (!chan_properties || !chan_properties->offset_gain_numerator ||
		!chan_properties->offset_gain_denominator || !adc_properties
		|| !adc_chan_result)
		return -EINVAL;

	xo_thm = pm8xxx_adc_scale_ratiometric_calib(adc_code,
			adc_properties, chan_properties);
	xo_thm <<= 4;
	pm8xxx_adc_map_linear(adcmap_ntcg_104ef_104fb,
		ARRAY_SIZE(adcmap_ntcg_104ef_104fb),
		xo_thm, &adc_chan_result->physical);

	return 0;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_tdkntcg_therm);

int32_t pm8xxx_adc_batt_scaler(struct pm8xxx_adc_arb_btm_param *btm_param,
		const struct pm8xxx_adc_properties *adc_properties,
		const struct pm8xxx_adc_chan_properties *chan_properties)
{
	int rc;

	rc = pm8xxx_adc_map_linear(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(btm_param->low_thr_temp),
		&btm_param->low_thr_voltage);
	if (rc)
		return rc;

	btm_param->low_thr_voltage *=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy;
	do_div(btm_param->low_thr_voltage, adc_properties->adc_vdd_reference);
	btm_param->low_thr_voltage +=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd;

	rc = pm8xxx_adc_map_linear(
		adcmap_btm_threshold,
		ARRAY_SIZE(adcmap_btm_threshold),
		(btm_param->high_thr_temp),
		&btm_param->high_thr_voltage);
	if (rc)
		return rc;

	btm_param->high_thr_voltage *=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].dy;
	do_div(btm_param->high_thr_voltage, adc_properties->adc_vdd_reference);
	btm_param->high_thr_voltage +=
		chan_properties->adc_graph[ADC_CALIB_RATIOMETRIC].adc_gnd;


	return rc;
}
EXPORT_SYMBOL_GPL(pm8xxx_adc_batt_scaler);
