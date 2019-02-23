#include <stdio.h>

#include <osmocom/codec/codec.h>


static int
gen_table_fr(int *tbl)
{
	int i,j;

	tbl[0] = tbl[1] = tbl[2] = tbl[3] = -1;

	for (i=0; i<260; i++)
	{
		j = 4 + gsm610_bitorder[i];
		if (i < 50)
			tbl[j] = 0;	/* Class 1a */
		else if (i < 182)
			tbl[j] = 1;	/* Class 1b */
		else
			tbl[j] = 2;	/* Class 2 */
	}

	return GSM_FR_BYTES * 8;
}

static int
gen_table_efr(int *tbl)
{
	int i, j, k;

	tbl[0] = tbl[1] = tbl[2] = tbl[3] = -1;

	for (i=0; i<260; i++)
	{
		j = gsm660_bitorder[i];

		if (j < 71)
			k = j;
		else if (j < 73)
			k = 71;
		else if (j < 123)
			k = j - 2;
		else if (j < 125)
			k = 119;
		else if (j < 178)
			k = j - 4;
		else if (j < 180)
			k = 172;
		else if (j < 230)
			k = j - 6;
		else if (j < 232)
			k = 222;
		else if (j < 252)
			k = j - 8;
		else
			continue;

		if (i < 50)
			tbl[k] = 0;	/* Class 1a */
		else if (i < 182)
			tbl[k] = 1;	/* Class 1b */
		else
			tbl[k] = 2;	/* Class 2 */
	}

	return GSM_EFR_BYTES * 8;
}

static int
gen_table_amr_12_2(int *tbl)
{
	int i;

	for (i=0; i<16; i++)
		tbl[i] = -1;
	
	for (i=0; i<244; i++)
		tbl[i+16] = i < 81 ? 0 : 1;
	
	for (i=0; i<4; i++)
		tbl[i+16+244] = -1;

	return 8 * 33;
}


static void
print_table(const char  *name, int *tbl, int len)
{
	int i;

	printf("static const int %s[] = {\n", name);

	for (i=0; i<len; i++)
	{
		if ((i & 15) == 0)
			printf("\t");

		printf("%2d", tbl[i]);
		
		if (((i & 15) == 15) || (i == len-1))
			printf(",\n");
		else
			printf(", ");
	}

	printf("};\n\n");
}


int main(int argc, char *argv[])
{
	int tbl[33*8];
	int rv;

	rv = gen_table_fr(tbl);
	print_table("gsm_fr_bitclass", tbl, rv);

	rv = gen_table_efr(tbl);
	print_table("gsm_efr_bitclass", tbl, rv);

	rv = gen_table_amr_12_2(tbl);
	print_table("gsm_amr_12_2_bitclass", tbl, rv);

	return 0;
}
