#include<iostream>
#include<stdlib.h>
#include<math.h>
#include<Windows.h>

#define MAXLENGTH 200000
#define Byte unsigned char//1字节
#define TByte unsigned short int//2字节
#define STByte signed short int
#define UByte unsigned int
#define SUByte signed int 

//jpeg2bmp
#define BYTE unsigned char
#define WORD unsigned short int

#define DWORD unsigned int
#define SDWORD signed int

#define SBYTE signed char
#define SWORD signed short int

//常用标记
#define SOI 0xD8
#define EOI 0xD9
#define SOS 0xDA
#define DQT 0xDB
#define DNL 0xDC
#define DRI 0xDD
#define DHT 0xC4


//帧开始标志
//huffman编码
#define SOF0 0xC0//基本DCT
#define SOF1 0xC1//扩展顺序DCT
#define SOF2 0xC2//累进DCT
#define SOF3 0xC3//无失真过程

#define SOF5 0xC5//差分顺序DCT
#define SOF6 0xC6//差分累进DCT
#define SOF7 0xC7//差分无失真过程

//算数编码
#define SOF9 0xC9
#define SOF10 0xCA
#define SOF11 0xCB

#define SOF13 0xCD
#define SOF14 0xCE
#define SOF15 0xCF


// IDCT parameters
#define  C1 0.9808
#define  C2 0.9239
#define  C3 0.8315
#define  C4 0.7071
#define  C5 0.5556
#define  C6 0.3827
#define  C7 0.1951

static TByte *QT[4];// 4-th quantization table
static Byte zigzag[64] = { 0, 1, 5, 6, 14, 15, 27, 28,
2, 4, 7, 13, 16, 26, 29, 42,
3, 8, 12, 17, 25, 30, 41, 43,
9, 11, 18, 24, 31, 40, 44, 53,
10, 19, 23, 32, 39, 45, 52, 54,
20, 22, 33, 38, 46, 51, 55, 60,
21, 34, 37, 47, 50, 56, 59, 61,
35, 36, 48, 49, 57, 58, 62, 63 };

static TByte *QT[4];
static STByte *DCT_coeff;
static STByte *DCT_coeff_IQ;
static STByte *DCT_coeff_temp;
static STByte YIDCT[64], CbIDCT[64], CrIDCT[64];
static Byte YQ_num;
static Byte CrQ_num;
static Byte CbQ_num;
static Byte *im_buffer;
static UByte X_image_bytes;
static double *Crpart_for_R;  //to reduce the calculate processes when turn YCrCb to RGB
static double *Cbpart_for_B;
static double *CbCrpart_for_G;

/*//jpeg2bmp
static BYTE *rlimit_table;
static SWORD Cr_tab[256], Cb_tab[256]; // Precalculated Cr, Cb tables
static SWORD Cr_Cb_green_tab[65536];

//end*/
Byte *buffer;

struct BitPlace{
	TByte Addr;
	Byte BitNumber;
};
BitPlace Current_Bit_Place;//待读的bit的位置

Byte QT_8bits[4][64];


TByte QT_16bits[4][64];


struct Huffman_Table{
	Byte Number[16];
	TByte TotalNumber;
	Byte *Weight;
	TByte Addr[16];
	TByte min_code[16];
	TByte max_code[16];
};

Huffman_Table HT_DC[2],HT_AC[2];
void init_HT(Huffman_Table *HT)
{
	HT->Weight = (Byte *)malloc(HT->TotalNumber*sizeof(Byte));
}

void build_HT(Huffman_Table *HT)
{
	int i = 0;
	while (i < 16 && 0 == HT->Number[i])
	{
		HT->Addr[i] = HT->TotalNumber + 1;
		HT->min_code[i] = 0xFFFF;
		HT->max_code[i] = 0x0000;
		i++;
	}
	if (i < 16)
	{
		HT->Addr[i] = 0;
		HT->min_code[i] = 0x0000;
		HT->max_code[i] = HT->min_code[i] + HT->Number[i] - 1;
		int last = i;
		i++;
		for (; i < 16; i++)
		{
			if (0 == HT->Number[i])
			{
				HT->Addr[i] = HT->TotalNumber + 1;
				HT->min_code[i] = 0xFFFF;
				HT->max_code[i] = 0x0000;
			}
			else
			{
				HT->Addr[i] = HT->Addr[last]+HT->Number[last];
				HT->min_code[i] = (HT->max_code[last]+1)<<(i-last);
				HT->max_code[i] = HT->min_code[i] + HT->Number[i] - 1;
				last = i;
			}
		}
	}

}


struct image{
	TByte rows;
	TByte cols;
	STByte **data;
};
image Y_image, Cr_image, Cb_image;

TByte Y_round, X_round;

struct Color{
	Byte H_sample;
	Byte V_sample;
	Byte QT_number;
	Byte HT_DC_number;
	STByte pre_DC;
	Byte HT_AC_number;
};

Color ColorsInfo[3],Y,Cb,Cr;

int Restart_maker;
TByte MCU_resrtart_th;

long data_place;
Byte get_byte(long *place)
{
	Byte Byte_value = buffer[*place];
	*place++;
	return Byte_value;
}


int read_JPG_Header(Byte *buffer){
	
	int Lq, Pq, Tq, DQT_end;//DQT
	int Lh, end_place;//DHT
	int Lr;//DRI
	int Lf, P, Nf;//SOF0;
	int Ls, Ns, tmp_place;//SOS
	TByte tmp_sum;

	long place = 0;
	data_place = 0;
	//通过SOI标记，确定是否是JPEG文件
	while (buffer[place] == 0xFF && buffer[place + 1] == 0xFF)
	{
		place++;
	}
	if (buffer[place] != 0xFF || buffer[place+1]!=SOI)
	{
		printf("这不是JPG文件！\n");
		system("pause");
		return 0;
	}
	place += 2;
	
	int SOS_found = 0, SOF_found = 0;
	Restart_maker = 0;
	while (!(buffer[place] == 0xFF && buffer[place + 1] == EOI) && SOS_found != 1)
	{
		if (buffer[place] != 0xFF)
		{
			place++;
			continue;
		} 
		place++;
		switch (buffer[place])
		{
		case DQT://读取量化表
			Lq = (buffer[place + 1] << 8) + buffer[place + 2];
			DQT_end = place + Lq;
			place += 3;
			while (place <= DQT_end)
			{
				Pq = buffer[place] >> 4;
				Tq = buffer[place] & 0x0F;
				place++;
				if (!Pq)//量化精度为8位
				{
					for (int j = 0; j < 64; j++)
					{
						QT_8bits[Tq][j] = buffer[place];
						place++;
					}
				}
				else//量化精度为16位
				{
					for (int j = 0; j < 64; j++)
					{
						QT_16bits[Tq][j] = (buffer[place] << 8) + buffer[place + 1];
						place += 2;
					}
				}
			}
			break;
		case DHT://读取huffman表
			Lh = (buffer[place + 1] << 8) + buffer[place + 2];
			end_place = place + Lh;
			place += 3;
			while (place<=end_place)
			{
				int DHT_Info = buffer[place];
				if (DHT_Info == 0x00)
				{
					place++;
					HT_DC[0].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_DC[0].Number[i] = buffer[place];
						HT_DC[0].TotalNumber += HT_DC[0].Number[i];
						place++;
					}
					init_HT(&HT_DC[0]);
					for (int i = 0; i < HT_DC[0].TotalNumber; i++)
					{
						HT_DC[0].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_DC[0]);
				}
				else if (DHT_Info == 0x01)
				{
					place++;
					HT_DC[1].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_DC[1].Number[i] = buffer[place];
						HT_DC[1].TotalNumber += HT_DC[1].Number[i];
						place++;
					}
					init_HT(&HT_DC[1]);
					for (int i = 0; i < HT_DC[1].TotalNumber; i++)
					{
						HT_DC[1].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_DC[1]);
				}
				else if (DHT_Info == 0x10)
				{
					place++;
					HT_AC[0].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_AC[0].Number[i] = buffer[place];
						HT_AC[0].TotalNumber += HT_AC[0].Number[i];
						place++;
					}
					init_HT(&HT_AC[0]);
					for (int i = 0; i < HT_AC[0].TotalNumber; i++)
					{
						HT_AC[0].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_AC[0]);
				}
				else if (DHT_Info == 0x11)
				{
					place++;
					HT_AC[1].TotalNumber = 0;
					for (int i = 0; i < 16; i++)
					{
						HT_AC[1].Number[i] = buffer[place];
						HT_AC[1].TotalNumber += HT_AC[1].Number[i];
						place++;
					}
					init_HT(&HT_AC[1]);
					for (int i = 0; i < HT_AC[1].TotalNumber; i++)
					{
						HT_AC[1].Weight[i] = buffer[place];
						place++;
					}
					build_HT(&HT_AC[1]);
				}
				else 
					continue;
			}
			break;
		case DRI:
			Lr = (buffer[place + 1] << 8) + buffer[place + 2];
			MCU_resrtart_th = (buffer[place + 3] << 8) + buffer[place + 4];
			if (0 == MCU_resrtart_th)
			{
				Restart_maker = 0;
			}
			else
			{
				Restart_maker = 1;
			}
			place = place + Lr + 1;
			break;
		case SOF0:
			Lf = (buffer[place + 1] << 8) + buffer[place + 2];
			P = buffer[place + 3];
			if (P != 8)
			{
				printf("只支持8位精度！\n");
				system("pause");
				return 0;
			}
			place += 4;
			Y_image.rows = (buffer[place] << 8) + buffer[place + 1];
			Y_image.cols = (buffer[place + 2] << 8) + buffer[place + 3];
			Nf = buffer[place+4];
			if (Nf != 3)
			{
				printf("只支持3个颜色分量的彩色图片！\n");
				system("pause");
				return 0;
			}
			place +=5;
			for (int i = 0; i < 3; i++)
			{
				int Ci = buffer[place];
				if (Ci < 1 || Ci > 3)
				{
					printf("只支持颜色分量编号1-3的彩色图片！\n");
					system("pause");
					return 0;
				}
				
				ColorsInfo[Ci - 1].H_sample = buffer[place + 1] >> 4;
				ColorsInfo[Ci - 1].V_sample = buffer[place + 1] & 0x0F;
				ColorsInfo[Ci - 1].QT_number = buffer[place + 2];
				place += 3;
			}
			Y = ColorsInfo[0];
			Cb = ColorsInfo[1];
			Cr = ColorsInfo[2];
			SOF_found = 1;
			break;
		case SOF1:
		case SOF2:
		case SOF3: 
		case SOF5:
		case SOF6:
		case SOF7:
		case SOF9:
		case SOF10:
		case SOF11:
		case SOF13:
		case SOF14:
		case SOF15: 
			printf("暂不支持除基本DCT（SOF0）外的模式\n"); 
			system("pause");
			return 0;
		
		case SOS:
			Ls = (buffer[place + 1] << 8) + buffer[place + 2];
			Ns = buffer[place + 3];
			if (Ns != 3)
			{
				printf("不合理的SOS标记！\n");
				system("pause");
				return 0;
			}
			tmp_place = place + 4;
			for (int i = 0; i < 3; i++)
			{
				int Cs = buffer[tmp_place];
				if (Cs < 1 || Cs>3)
				{
					printf("SOS头标中的分量设置不合理！\n");
					system("pause");
					return 0;
				}
				ColorsInfo[Cs - 1].HT_DC_number = buffer[tmp_place + 1] >> 4;
				ColorsInfo[Cs - 1].HT_AC_number = buffer[tmp_place + 1] & 0x0F;
				tmp_place += 2;
			}
			Y = ColorsInfo[0];
			Cr = ColorsInfo[1];
			Cb = ColorsInfo[2];
			SOS_found = 1;
			place = place + Ls + 1;
			data_place = place;
			Current_Bit_Place.Addr = data_place;
			Current_Bit_Place.BitNumber = 7;//该字节的最高位
			break;
		default: place++; break;
		}
	}
	if (!SOS_found)
	{
		printf("没有发现SOS标记！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}
	if (!SOF_found)
	{
		printf("没有发现SOF标记！\n");
		printf("头文件解码结束后的位置为：%x\n", place);
		system("pause");
		return 0;
	}

	if (Y_image.rows % (Y.V_sample * 8) == 0)
		Y_round = Y_image.rows;
	else
		Y_round = (Y_image.rows / (Y.V_sample * 8) + 1)*(Y.V_sample * 8);
	if (Y_image.cols % (Y.H_sample * 8) == 0)
		X_round = Y_image.cols;
	else
		X_round = (Y_image.cols / (Y.H_sample * 8) + 1)*(Y.H_sample * 8);

	printf("头文件解码结束后的位置为：%x\n", place);
	im_buffer = (BYTE *)malloc(X_round * Y_round * 3); //channel = 3,addtion
	if (im_buffer == NULL) printf("Not enough memory for storing the JPEG image");
	return 1;
}




TByte get_1_bit()
{
	TByte BitValue;
	BitValue = (buffer[Current_Bit_Place.Addr] >> Current_Bit_Place.BitNumber) & 0x01;
	if (0 == Current_Bit_Place.BitNumber)
	{
		Current_Bit_Place.Addr++;
		Current_Bit_Place.BitNumber = 7;
	}
	else
	{
		Current_Bit_Place.BitNumber--;
	}
	return BitValue;
}


STByte tmp_8x8_unit[64];
int huffman_decode(Byte HT_DC_num,Byte HT_AC_num,STByte *pre_DC)
{
	TByte DC_code=0, AC_code=0;
	TByte DC_value = 0, AC_value = 0;
	Byte size;
	
	for (int i = 0; i < 64; i++)
	{
		tmp_8x8_unit[i] = 0;
	}

	int Weight_Place;

	//解码DC系数
	int DC_code_found = 0;
	for (int k = 0; k < 16; k++)
	{
		TByte tmp_Bit;
		tmp_Bit = get_1_bit();
		DC_code = (DC_code << 1) + tmp_Bit;
		if (DC_code >= HT_DC[HT_DC_num].min_code[k]&&DC_code <= HT_DC[HT_DC_num].max_code[k])
		{
			DC_code_found = 1;
			Weight_Place = HT_DC[HT_DC_num].Addr[k] + (DC_code - HT_DC[HT_DC_num].min_code[k]);
			size = HT_DC[HT_DC_num].Weight[Weight_Place];
			break;
		}
	}
	if (DC_code_found)
	{
		if (size > 15)
		{
			printf("DC系数太大，超过15位！\n");
			system("pause");
			return 0;
		}
		else if (0 == size)
		{
			tmp_8x8_unit[0] = 0;
		}
		else
		{
			for (int i = 0; i < size; i++)
			{
				TByte tmp_Bit;
				tmp_Bit = get_1_bit();
				DC_value = (DC_value << 1) + tmp_Bit;
			}
			int sign;
			sign = (DC_value >> (size - 1));
			if (1 == sign)
			{
				tmp_8x8_unit[0] = DC_value;
			}
			else
			{
				int tmp = 0x00;
				for (int i = 0; i < size; i++)
				{
					tmp = (tmp << 1) + 1;
				}
				tmp_8x8_unit[0] = 0-((~DC_value) & tmp);

			}
		}

		tmp_8x8_unit[0] += *pre_DC;
		*pre_DC = tmp_8x8_unit[0];
	}
	else
	{
		printf("没有找到DC_code!\n");
		system("pause");
		return 0;
	}

	//解码AC系数
	int j = 1;
	int EOB_found = 0;
	int RunLegnth;

	while ((j < 64) && (!EOB_found))
	{
		AC_code = 0;
		int AC_code_found=0;
		for (int k = 0; k < 16; k++)
		{
			TByte tmp_Bit;
			tmp_Bit = get_1_bit();
			AC_code = (AC_code << 1) + tmp_Bit;
			if (AC_code >= HT_AC[HT_AC_num].min_code[k] && AC_code <= HT_AC[HT_AC_num].max_code[k])
			{
				AC_code_found = 1;
				Weight_Place = HT_AC[HT_AC_num].Addr[k] + (AC_code - HT_AC[HT_AC_num].min_code[k]);
				RunLegnth = HT_AC[HT_AC_num].Weight[Weight_Place] >> 4;
				j += RunLegnth;
				size = HT_AC[HT_AC_num].Weight[Weight_Place] & 0x0f;
				break;
			}
		}
		if (AC_code_found)
		{
			if (0 == size)
			{
				if (0 == RunLegnth)
					EOB_found = 1;
				else
				{
					tmp_8x8_unit[j] = 0;
					j++;
				}

			}
			else
			{
				for (int i = 0; i < size; i++)
				{
					TByte tmp_Bit;
					tmp_Bit = get_1_bit();
					AC_value = (AC_value << 1) + tmp_Bit;
				}
				int sign;
				sign = (AC_value >> (size - 1));
				if (1 == sign)
				{
					tmp_8x8_unit[j] = AC_value;
				}
				else
				{
					int tmp = 0x00;
					for (int i = 0; i < size; i++)
					{
						tmp = (tmp << 1) + 1;
					}
					tmp_8x8_unit[j] = 0 - ((~AC_value) & tmp);
				}
				j++;
			}
		}
		else
		{
			printf("没有找到AC_code!\n");
			system("pause");
			return 0;
		}
		
	}

}




// IQuntization
void init_quntization() {
	int i = 0;
	for (i = 0; i <= 3; i++)
	{
		QT[i] = (TByte *)malloc(sizeof(TByte) * 64);
	}

}
void init_IDCT() {
	DCT_coeff = (STByte *)malloc(sizeof(STByte) * 64);
	DCT_coeff_IQ = (STByte *)malloc(sizeof(STByte) * 64);
	DCT_coeff_temp = (STByte *)malloc(sizeof(STByte) * 64);
}

/*void load_Qtable(WORD *Qtable) {
	float scalefactor[8] = { 1.0f, 1.387039845f, 1.306562965f, 1.175875602f,
		1.0f, 0.785694958f, 0.541196100f, 0.275899379f };
	BYTE j, row, col;
	j = 0;
	for (row = 0; row < 63; row++) {
		for (col = 0; col <= 7; col++) {
			Qtable[j] *= scalefactor[row] * scalefactor[col];
			j++;
		}
	}
}*/

void IQuntization(STByte *input, STByte *output, Byte Q) {
	TByte *Qtable, *Qtable_izigzag;
	Qtable_izigzag = (TByte *)malloc(sizeof(TByte) * 64);
	Qtable = QT[Q];
	for (int i = 0; i < 64; i++) {
		Qtable_izigzag[i] = Qtable[zigzag[i]];//Qtable do zigzaging
											  //printf("Q table:%d\n", Qtable[i]);
		for (int i = 0; i < 64; i++)
			output[i] = input[i] * Qtable_izigzag[i];
	}
}

//IDCT in rows
void IDCT_row(STByte *row_start, double *row_idct) {
	double tmp[16];
	//first step
	tmp[0] = row_start[0] * C4 + row_start[2] * C2;
	tmp[1] = row_start[4] * C4 + row_start[6] * C6;
	tmp[2] = row_start[0] * C4 + row_start[2] * C6;
	tmp[3] = -row_start[4] * C4 - row_start[6] * C2;
	tmp[4] = row_start[0] * C4 - row_start[2] * C6;
	tmp[5] = -row_start[4] * C4 + row_start[6] * C2;
	tmp[6] = row_start[0] * C4 - row_start[2] * C2;
	tmp[7] = row_start[4] * C4 - row_start[6] * C6;

	tmp[8] = row_start[1] * C7 - row_start[3] * C5;
	tmp[9] = row_start[5] * C3 - row_start[7] * C1;
	tmp[10] = row_start[1] * C5 - row_start[3] * C1;
	tmp[11] = row_start[5] * C7 + row_start[7] * C3;
	tmp[12] = row_start[1] * C3 - row_start[3] * C7;
	tmp[13] = -row_start[5] * C1 - row_start[7] * C5;
	tmp[14] = row_start[1] * C1 + row_start[3] * C3;
	tmp[15] = row_start[5] * C5 + row_start[7] * C7;
	//second step
	tmp[0] = 0.5*(tmp[0] + tmp[1]);
	tmp[1] = 0.5*(tmp[2] + tmp[3]);
	tmp[2] = 0.5*(tmp[4] + tmp[5]);
	tmp[3] = 0.5*(tmp[6] + tmp[7]);
	tmp[4] = 0.5*(tmp[8] + tmp[9]);
	tmp[5] = 0.5*(tmp[10] + tmp[11]);
	tmp[6] = 0.5*(tmp[12] + tmp[13]);
	tmp[7] = 0.5*(tmp[14] + tmp[15]);
	//third step
	row_idct[0] = tmp[0] + tmp[7];
	row_idct[1] = tmp[1] + tmp[6];
	row_idct[2] = tmp[2] + tmp[5];
	row_idct[3] = tmp[3] + tmp[4];
	row_idct[4] = tmp[3] - tmp[4];
	row_idct[5] = tmp[2] - tmp[5];
	row_idct[6] = tmp[1] - tmp[6];
	row_idct[7] = tmp[0] - tmp[7];
}

// IDCT in cols
void IDCT_col(double *col_start, STByte *idct_output)
{
	double tmp[16];
	idct_output = (STByte*)malloc(sizeof(STByte) * 64);
	//first step
	tmp[0] = col_start[0 * 8] * C4 + col_start[2 * 8] * C2;
	tmp[1] = col_start[4 * 8] * C4 + col_start[6 * 8] * C6;
	tmp[2] = col_start[0 * 8] * C4 + col_start[2 * 8] * C6;
	tmp[3] = -col_start[4 * 8] * C4 - col_start[6 * 8] * C2;
	tmp[4] = col_start[0 * 8] * C4 - col_start[2 * 8] * C6;
	tmp[5] = -col_start[4 * 8] * C4 + col_start[6 * 8] * C2;
	tmp[6] = col_start[0 * 8] * C4 - col_start[2 * 8] * C2;
	tmp[7] = col_start[4 * 8] * C4 - col_start[6 * 8] * C6;

	tmp[8] = col_start[1 * 8] * C7 - col_start[3 * 8] * C5;
	tmp[9] = col_start[5 * 8] * C3 - col_start[7 * 8] * C1;
	tmp[10] = col_start[1 * 8] * C5 - col_start[3 * 8] * C1;
	tmp[11] = col_start[5 * 8] * C7 + col_start[7 * 8] * C3;
	tmp[12] = col_start[1 * 8] * C3 - col_start[3 * 8] * C7;
	tmp[13] = -col_start[5 * 8] * C1 - col_start[7 * 8] * C5;
	tmp[14] = col_start[1 * 8] * C1 + col_start[3 * 8] * C3;
	tmp[15] = col_start[5 * 8] * C5 + col_start[7 * 8] * C7;


	//second step
	tmp[0] = 0.5*(tmp[0] + tmp[1]);
	tmp[1] = 0.5*(tmp[2] + tmp[3]);
	tmp[2] = 0.5*(tmp[4] + tmp[5]);
	tmp[3] = 0.5*(tmp[6] + tmp[7]);
	tmp[4] = 0.5*(tmp[8] + tmp[9]);
	tmp[5] = 0.5*(tmp[10] + tmp[11]);
	tmp[6] = 0.5*(tmp[12] + tmp[13]);
	tmp[7] = 0.5*(tmp[14] + tmp[15]);

	//third step
	idct_output[0 * 8] = tmp[0] + tmp[7];
	idct_output[1 * 8] = tmp[1] + tmp[6];
	idct_output[2 * 8] = tmp[2] + tmp[5];
	idct_output[3 * 8] = tmp[3] + tmp[4];
	idct_output[4 * 8] = tmp[3] - tmp[4];
	idct_output[5 * 8] = tmp[2] - tmp[5];
	idct_output[6 * 8] = tmp[1] - tmp[6];
	idct_output[7 * 8] = tmp[0] - tmp[7];
}
void IDCT(STByte *input, STByte *output) {
	double middle_matrix[64];
	for (int i = 0; i < 8; i++) IDCT_row(&input[8 * i], &middle_matrix[8 * i]);
	for (int j = 0; j < 8; j++) {
		IDCT_col(&middle_matrix[j], &output[j]);
		//printf("%d\n", &output[i]);
	}
}

/*R = Y + 1.40200 * Cr + 128
G = Y - 0.34414 * Cb - 0.71414 * Cr + 128
B = Y + 1.77200 * Cb + 128 */
/*//jpeg2img

WORD WORD_hi_lo(BYTE byte_high, BYTE byte_low)
{
#if 0
	_asm {
		mov ah, byte_high
		mov al, byte_low
	}
#else
	return (((int)byte_high) << 8) | (byte_low);
#endif
}

void prepare_range_limit_table()
// Allocate and fill in the sample_range_limit table
{
	int j;
	rlimit_table = (BYTE *)malloc(5 * 256L + 128);
	// First segment of "simple" table: limit[x] = 0 for x < 0 
	memset((void *)rlimit_table, 0, 256);
	rlimit_table += 256;	//allow negative subscripts of simple table 
							//Main part of "simple" table: limit[x] = x 
	for (j = 0; j < 256; j++) rlimit_table[j] = j;
	// End of simple table, rest of first half of post-IDCT table 
	for (j = 256; j < 640; j++) rlimit_table[j] = 255;
	// Second half of post-IDCT table 
	memset((void *)(rlimit_table + 640), 0, 384);
	for (j = 0; j < 128; j++) rlimit_table[j + 1024] = j;
}

void convert_8x8_YCbCr_to_RGB(BYTE *Y, BYTE *Cb, BYTE *Cr, DWORD im_loc, DWORD X_image_bytes, BYTE *im_buffer)
// Functia (ca optimizare) poate fi apelata si fara parametrii Y,Cb,Cr
// Stim ca va fi apelata doar in cazul 1x1
{
	DWORD x, y;
	BYTE im_nr;
	BYTE *Y_val = Y, *Cb_val = Cb, *Cr_val = Cr;
	BYTE *ibuffer = im_buffer + im_loc;

	for (y = 0; y<8; y++)
	{
		im_nr = 0;
		for (x = 0; x<8; x++)
		{
			ibuffer[im_nr++] = rlimit_table[*Y_val + Cb_tab[*Cb_val]]; //B
			ibuffer[im_nr++] = rlimit_table[*Y_val + Cr_Cb_green_tab[WORD_hi_lo(*Cr_val, *Cb_val)]]; //G
			ibuffer[im_nr++] = rlimit_table[*Y_val + Cr_tab[*Cr_val]]; // R
																	   
																	   // Monochrome display
																	   im_buffer[im_nr++] = *Y_val;
																	   im_buffer[im_nr++] = *Y_val;
																	   im_buffer[im_nr++] = *Y_val;
																	   
			Y_val++; Cb_val++; Cr_val++; im_nr++;
		}
		ibuffer += X_image_bytes;
	}
}

void convert_8x8_YCbCr_to_RGB_tab(BYTE *Y, BYTE *Cb, BYTE *Cr, BYTE *tab, DWORD im_loc, DWORD X_image_bytes, BYTE *im_buffer)
// Functia (ca optimizare) poate fi apelata si fara parametrii Cb,Cr
{
	DWORD x, y;
	BYTE nr, im_nr;
	BYTE Y_val, Cb_val, Cr_val;
	BYTE *ibuffer = im_buffer + im_loc;

	nr = 0;
	for (y = 0; y<8; y++)
	{
		im_nr = 0;
		for (x = 0; x<8; x++)
		{
			Y_val = Y[nr];
			Cb_val = Cb[tab[nr]]; Cr_val = Cr[tab[nr]]; // reindexare folosind tabelul
														// de supraesantionare precalculat
			ibuffer[im_nr++] = rlimit_table[Y_val + Cb_tab[Cb_val]]; //B
			ibuffer[im_nr++] = rlimit_table[Y_val + Cr_Cb_green_tab[WORD_hi_lo(Cr_val, Cb_val)]]; //G
			ibuffer[im_nr++] = rlimit_table[Y_val + Cr_tab[Cr_val]]; // R
			nr++; im_nr++;
		}
		ibuffer += X_image_bytes;
	}
}
*/



/* zhangyi
void prepare_calculate_tables() {
	UByte k;
	UByte Cr_t, Cb_t;

	Cbpart_for_B = (double *)malloc(sizeof(double) * 256);
	Crpart_for_R = (double *)malloc(sizeof(double) * 256);
	CbCrpart_for_G = (double *)malloc(sizeof(double) * 65536);

	for (k = 0; k <= 255; k++) Crpart_for_R[k] = (double)(k * 1.402);  //this rotation takes a lot of time
																	   //printf("Done1!\n");
	for (k = 0; k <= 255; k++) Cbpart_for_B[k] = (double)(k * 1.772);
	//printf("Done2!\n");
	for (Cr_t = 0; Cr_t <= 255; Cr_t++)
		for (Cb_t = 0; Cb_t <= 255; Cb_t++)
			CbCrpart_for_G[(Cr_t << 8) + Cb_t] = (double)(-0.34414 * Cb_t - 0.71414 * Cr_t);
	//printf("Done3!\n");
}

void convert_8x8_YCbCr_to_RGB(STByte *Y, STByte *Cb, STByte *Cr, UByte location, UByte X_image_bytes, Byte *im_buffer) {
	Byte *iBuffer;
	UByte x, y;
	STByte Y_data, Cb_data, Cr_data;  //to be put into the temporary Y Cb Cr value
	STByte B_temp, G_temp, R_temp;
	UByte Cr_Cb;
	Byte nr, im_nr;  //nr is used to read each elements of Y Cr Cb, im_nr is the sequence of im_buffer

	iBuffer = im_buffer + location;
	nr = 0;
	for (y = 0; y < 8; y++) {
		im_nr = 0; //for every line begin with the first element
		for (x = 0; x < 8; x++) {
			Y_data = Y[nr];  Cb_data = Cb[nr];  Cr_data = Cr[nr];
			Cr_Cb = (((UByte)Cr_data) << 8) + (UByte)Cb_data;

			B_temp = (STByte)(Y_data + Cbpart_for_B[Cb_data]) + 128;  //B
			if (B_temp > 255)  iBuffer[im_nr++] = (Byte)255;
			else if (B_temp < 0) iBuffer[im_nr++] = (Byte)0;
			else iBuffer[im_nr++] = (Byte)B_temp;
			//printf("%d ", iBuffer[im_nr - 1]);

			G_temp = (STByte)(Y_data + CbCrpart_for_G[Cr_Cb]) + 128;  //G
			if (G_temp > 255)  iBuffer[im_nr++] = (Byte)255;
			else if (G_temp < 0) iBuffer[im_nr++] = (Byte)0;
			else iBuffer[im_nr++] = (Byte)G_temp;
			//printf("%d ", iBuffer[im_nr - 1]);

			R_temp = (STByte)(Y_data + Crpart_for_R[Cr_data]) + 128;  //R
			if (R_temp > 255)  iBuffer[im_nr++] = (Byte)255;
			else if (R_temp < 0) iBuffer[im_nr++] = (Byte)0;
			else iBuffer[im_nr++] = (Byte)R_temp;

			//printf("%d\n", iBuffer[im_nr - 1]);
			nr++; //im_nr++; //to make space for A's data
		}
		iBuffer += X_image_bytes;  //to the next line
	}
	//im_loc += im_nr;
}

end*/


void convert_YCbCr_to_RGB(STByte *Y, STByte *Cb, STByte *Cr, UByte location, UByte image_bytes, Byte *im_buffer) {
	Byte *ibuffer= im_buffer + location;
	STByte Y_temp, Cb_temp, Cr_temp;
	STByte B_temp, G_temp, R_temp;
	//TByte Cr_Cb;
	Byte nr, im_nr;// nr is used for each element of YCrCb, im_nr is sequence of im_buffer
	//ibuffer = im_buffer + location;// adress of image buffer
	nr = 0;
	for (int i = 0; i < 8; i++) {
		im_nr = 0;// for every row begin with the first element
		for (int j = 0; j < 8; j++) {
			Y_temp = Y[nr]; Cb_temp = Cb[nr]; Cr_temp = Cr[nr];
			// the sequnce of saving image is B-G-R
			B_temp = (short)(Y_temp + 1.77200*Cb_temp) + 128;  //B
			if (B_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (B_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)B_temp;

			G_temp = (short)(Y_temp - 0.34414*Cb_temp - 0.71414*Cr_temp) + 128;  //G
			if (G_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (G_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)G_temp;

			R_temp = (short)(Y_temp + 1.40200*Cr_temp) + 128;  //R
			if (R_temp > 255)  ibuffer[im_nr++] = (Byte)255;
			else if (R_temp < 0) ibuffer[im_nr++] = (Byte)0;
			else ibuffer[im_nr++] = (Byte)R_temp;

			nr++;
		}
		ibuffer += image_bytes;// to the next row
	}
	
}


void restart()
{
	Y.pre_DC = 0; 
	Cb.pre_DC = 0;
	Cr.pre_DC = 0;
	init_IDCT();
}

void decode_MCU_1x1(UByte im_loc)
{
	huffman_decode(Y.HT_DC_number, Y.HT_AC_number, &Y.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, YQ_num);
	IDCT(DCT_coeff_IQ, YIDCT);
	//IQ(Y);
	//IDCT(Y);
	huffman_decode(Cr.HT_DC_number, Cr.HT_AC_number, &Cr.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, CrQ_num);
	IDCT(DCT_coeff_IQ, CrIDCT);
	//IQ(Cr);
	//IDCT(Cr);
	huffman_decode(Cb.HT_DC_number, Cb.HT_AC_number, &Cb.pre_DC);
	IQuntization(DCT_coeff, DCT_coeff_IQ, CbQ_num);
	IDCT(DCT_coeff_IQ, CbIDCT);
	//IQ(Cb);
	//IDCT(Cb);
	convert_YCbCr_to_RGB(YIDCT, CbIDCT, CrIDCT, im_loc, X_image_bytes, im_buffer);
	//convert_8x8_YCbCr_to_RGB(YIDCT, CbIDCT, CrIDCT, im_loc, X_image_bytes, im_buffer);
}


long getfilesize(FILE *fp)
{
	long filesize;
	fseek(fp, 0, SEEK_END);
	filesize = ftell(fp);
	return filesize;
}

/*
int main()
{
	char filename[] = "input.jpg";
	FILE *fp;
	errno_t err;
	UByte im_loc=0;


	//读入文件大小
	if ((err = fopen_s(&fp, filename, "rb")) != 0)
	{
		printf("输入的jpg文件不能正常打开！");
		fclose(fp);
		system("pause");
		return err;
	}
	long JPG_length = getfilesize(fp);
	fclose(fp);
	
	////读入jpg文件，存入缓存中
	if ((err = fopen_s(&fp, filename, "rb")) != 0)
	{
		printf("输入的jpg文件不能正常打开！");
		fclose(fp);
		system("pause");
		return err;
	}
	buffer = (Byte *)malloc(JPG_length);
	long read_result = fread_s(buffer, JPG_length, 1, JPG_length, fp);
	fclose(fp);
	if (read_result< JPG_length)
	{
		printf("读取文件的时候出现了问题！\n");
		system("pause");
		return 0;
	}
	
	read_JPG_Header(buffer);
	restart();
	huffman_decode(Y.HT_DC_number, Y.HT_AC_number, &Y.pre_DC);
	decode_MCU_1x1(im_loc);
	
	fclose(fp);
	return 0;
}
*/
int main() {
	STByte *input, *output;
	output = (STByte*)malloc(sizeof(STByte*) * 64);
	STByte a[64] = { 2.8284 ,   2.8284 ,   2.8284   , 2.8284 ,   2.8284 ,   2.8284 ,   2.8284,    2.8284,
		0  ,       0  ,       0    ,     0    ,     0    ,     0    ,     0   ,      0,
		0 ,        0  ,       0   ,      0   ,      0    ,     0   ,     0     ,     0,
		0  ,       0  ,       0   ,      0   ,      0    ,     0   ,      0    ,     0,
		0  ,       0  ,       0    ,     0   ,      0   ,      0   ,      0    ,     0,
		0  ,       0  ,       0   ,      0    ,     0    ,     0   ,      0    ,     0,
		0  ,       0  ,       0    ,     0   ,      0   ,      0   ,      0    ,     0,
		0  ,       0  ,       0   ,      0    ,     0    ,     0   ,      0    ,     0
	};
	/*STByte a[8] =
	{ 12.7279, -6.4423 ,        0, -0.6735 ,        0, -0.2009 ,        0, -0.0507 };*/
	input =&a[0];

	IDCT(input, output);
	for (int i = 0; i < 64; i++) { printf("%d\n", output); output++; }
	system("pause");
}