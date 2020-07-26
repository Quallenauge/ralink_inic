#include "rlk_inic.h"
#include "rlk_inic_def.h"
#include <linux/firmware.h>

unsigned char BtoH(char ch)
{
	if (ch >= '0' && ch <= '9')	return(ch - '0');		 // Handle numerals
	if (ch >= 'A' && ch <= 'F')	return(ch - 'A' + 0xA);	 // Handle capitol hex digits
	if (ch >= 'a' && ch <= 'f')	return(ch - 'a' + 0xA);	 // Handle small hex digits
	return(255);
}

void AtoH(char * src, unsigned char *dest, int destlen)
{
	char * srcptr;
	unsigned char *destTemp;

	srcptr = src;   
	destTemp = (unsigned char *) dest; 

	while (destlen--)
	{
		*destTemp = BtoH(*srcptr++) << 4;	 // Put 1st ascii byte in upper nibble.
		*destTemp += BtoH(*srcptr++);	   // Add 2nd ascii byte to above.
		destTemp++;
	}
}

#if 0
// We assume the s1 is a sting, s2 is a memory space with 6 bytes. and content of s1 will be changed.
static boolean rtstrmactohex(char *s1, char *s2)
{
	int i = 0;
	char *ptokS = s1, *ptokE = s1;

	if (strlen(s1) != ETH_MAC_ADDR_STR_LEN)
		return FALSE;

	while ((*ptokS) != '\0')
	{
		if ((ptokE = strchr(ptokS, ':')) != NULL)
			*ptokE++ = '\0';
		if ((strlen(ptokS) != 2) || (!isxdigit(*ptokS)) || (!isxdigit(*(ptokS+1))))
			break; // fail
		AtoH(ptokS, &s2[i++], 1);
		ptokS = ptokE;
		if (i == 6)
			break; // parsing finished
	}

	return( i == 6 ? TRUE : FALSE);

}

// we assume the s1 and s2 both are strings.
static boolean rtstrcasecmp(char *s1, char *s2)
{
	char *p1 = s1, *p2 = s2;

	if (strlen(s1) != strlen(s2))
		return FALSE;

	while (*p1 != '\0')
	{
		if ((*p1 != *p2) && ((*p1 ^ *p2) != 0x20))
			return FALSE;
		p1++;
		p2++;
	}

	return TRUE;
}
#endif


#ifdef MULTIPLE_CARD_SUPPORT
// we assume the s1 (buffer) and s2 (key) both are strings.
char * rtstrstruncasecmp(char * s1, char * s2)
{
	s32 l1, l2, i;
	s8 temp1, temp2;

	l2 = strlen(s2);
	if (!l2)
		return(char *) s1;

	l1 = strlen(s1);

	while (l1 >= l2)
	{
		l1--;

		for (i=0; i<l2; i++)
		{
			temp1 = *(s1+i);
			temp2 = *(s2+i);

			if (('a' <= temp1) && (temp1 <= 'z'))
				temp1 = 'A'+(temp1-'a');
			if (('a' <= temp2) && (temp2 <= 'z'))
				temp2 = 'A'+(temp2-'a');

			if (temp1 != temp2)
				break;
		}

		if (i == l2)
			return(char *) s1;

		s1++;
	}

	return NULL; // not found
}

#endif

/**
 * strstr - Find the first substring in a %NUL terminated string
 * @s1: The string to be searched
 * @s2: The string to search for
 */
static char * rtstrstr(const char * s1,const char * s2)
{
	int l1, l2;

	l2 = strlen(s2);
	if (!l2)
		return(char *) s1;

	l1 = strlen(s1);

	while (l1 >= l2)
	{
		l1--;
		if (!memcmp(s1,s2,l2))
			return(char *) s1;
		s1++;
	}

	return NULL;
}

/**
 * rstrtok - Split a string into tokens
 * @s: The string to be searched
 * @ct: The characters to search for
 * * WARNING: strtok is deprecated, use strsep instead. However strsep is not compatible with old architecture.
 */
static char * __rstrtok;
static char * rstrtok(char * s,const char * ct)
{
	char *sbegin, *send;

	sbegin  = s ? s : __rstrtok;
	if (!sbegin)
	{
		return NULL;
	}

	sbegin += strspn(sbegin,ct);
	if (*sbegin == '\0')
	{
		__rstrtok = NULL;
		return( NULL );
	}

	send = strpbrk( sbegin, ct);
	if (send && *send != '\0')
		*send++ = '\0';

	__rstrtok = send;

	return(sbegin);
}

/*
	========================================================================

	Routine Description:
		Find key section for Get key parameter.

	Arguments:
		buffer                      Pointer to the buffer to start find the key section
		section                     the key of the secion to be find

	Return Value:
		NULL                        Fail
		Others                      Success
	========================================================================
*/
static unsigned char *  profile_find_section(const char  *buffer)
{
	char temp_buf[32];
	unsigned char *  ptr;

	RLK_STRCPY(temp_buf, "Default\n");

	if ((ptr = rtstrstr(buffer, temp_buf)) != NULL){
		return ptr;
	}else{
		return NULL;
	}
}

/*
	========================================================================

	Routine Description:
		Get key parameter.

	Arguments:
		key                         Pointer to key string
		dest                        Pointer to destination      
		destsize                    The datasize of the destination
		buffer                      Pointer to the buffer to start find the key

	Return Value:
		TRUE                        Success
		FALSE                       Fail

	Note:
		This routine get the value with the matched key (case case-sensitive)
	========================================================================
*/
int profile_get_keyparameter(
							const char *   key,
							char *   dest,   
							int     destsize,
							const char *   buffer)
{
	unsigned char *temp_buf1 = NULL;
	unsigned char *temp_buf2 = NULL;
	char *start_ptr;
	char *end_ptr;
	char *ptr;
	char *offset = 0;
	int  len;
	int cat_len = 0;

	temp_buf1 = kmalloc(MAX_PARAM_BUFFER_SIZE, MEM_ALLOC_FLAG);


	if (temp_buf1 == NULL)
		return(FALSE);

	temp_buf2 = kmalloc(MAX_PARAM_BUFFER_SIZE, MEM_ALLOC_FLAG);

	if (temp_buf2 == NULL)
	{
		kfree(temp_buf1);
		return(FALSE);
	}

	//find section
	if ((offset = profile_find_section(buffer)) == NULL)
	{
		kfree(temp_buf1);
		kfree(temp_buf2);
		return(FALSE);
	}

	strncpy(temp_buf1, "\n", sizeof("\n"));
	cat_len = MAX_PARAM_BUFFER_SIZE - strlen(temp_buf1);
	strncat(temp_buf1, key, cat_len);
	cat_len = MAX_PARAM_BUFFER_SIZE - strlen(temp_buf1);
	strncat(temp_buf1, "=", cat_len);

	//search key
	if ((start_ptr=rtstrstr(offset, temp_buf1))==NULL)
	{
		kfree(temp_buf1);
		kfree(temp_buf2);
		return(FALSE);
	}

	start_ptr+=strlen("\n");
	if ((end_ptr=rtstrstr(start_ptr, "\n"))==NULL)
		end_ptr=start_ptr+strlen(start_ptr);

	if (end_ptr<start_ptr)
	{
		kfree(temp_buf1);
		kfree(temp_buf2);
		return(FALSE);
	}

	memmove(temp_buf2, start_ptr, end_ptr-start_ptr);
	temp_buf2[end_ptr-start_ptr]='\0';
	len = strlen(temp_buf2);
	strncpy(temp_buf1, temp_buf2, MAX_PARAM_BUFFER_SIZE);

	if ((start_ptr=rtstrstr(temp_buf1, "=")) == NULL)
	{
		kfree(temp_buf1);
		kfree(temp_buf2);
		return(FALSE);
	}

	strncpy(temp_buf2, start_ptr+1, MAX_PARAM_BUFFER_SIZE);
	ptr = temp_buf2;
	//trim space or tab
	while (*ptr != 0x00)
	{
		if ((*ptr == ' ') || (*ptr == '\t'))
			ptr++;
		else
			break;
	}

	len = strlen(ptr);    
	memset(dest, 0x00, destsize);
	strncpy(dest, ptr, len >= destsize ?  destsize: len);

	kfree(temp_buf1);
	kfree(temp_buf2);
	return TRUE;
}

void rlk_inic_read_profile(iNIC_PRIVATE *pAd)
{
	char                    *tmpbuf;
	char                    *profilebuf;
	unsigned char           *macptr;                            
	int                     i=0;

	const struct firmware *fw;
	char *pf_name;

	profilebuf = kcalloc(MAX_INI_BUFFER_SIZE, 1, MEM_ALLOC_FLAG);
	if (profilebuf == NULL)
	{
		dev_err(&pAd->dev->dev, "fail allocate profile buffer\n");
		return;
	}

	tmpbuf = kcalloc(MAX_PARAM_BUFFER_SIZE, 1, MEM_ALLOC_FLAG);
	if (tmpbuf == NULL)
	{
		dev_err(&pAd->dev->dev, "fail allocate buffer\n");
		kfree(profilebuf);
		return;
	}

#ifdef MULTIPLE_CARD_SUPPORT
	if (pAd->RaCfgObj.InterfaceNumber >= 0)
		pf_name = pAd->RaCfgObj.profile.read_name;
	else
#endif //MULTIPLE_CARD_SUPPORT
	if ((pAd->RaCfgObj.InterfaceNumber >= 0)&&(pAd->RaCfgObj.InterfaceNumber < CONCURRENT_CARD_NUM))
	{
		s8 idx = pAd->RaCfgObj.InterfaceNumber;
		pf_name = ConcurrentObj.Profile[idx].read_name;
	}
	else
	{
		if (pAd->RaCfgObj.opmode)
			pf_name = AP_PROFILE;
		else
			pf_name	= STA_PROFILE;
	}
	dev_err(&pAd->dev->dev, "Request profile: %s\n", pf_name);
	if (0 != request_firmware_into_buf(&fw, pf_name, &pAd->dev->dev, profilebuf, MAX_INI_BUFFER_SIZE*sizeof(char))) {
		dev_err(&pAd->dev->dev, "Failed to request profile: %s\n", pf_name);
		goto exit;
	}

	printk("Loaded firmware size: %d\n", fw->size);

	if (pAd->RaCfgObj.opmode)
	{
		// BssidNum; This must read first of other multiSSID field, so list this field first in configuration file
		if (profile_get_keyparameter("BssidNum", tmpbuf, 25, profilebuf))
		{
			pAd->RaCfgObj.BssidNum = (unsigned char) simple_strtol(tmpbuf, 0, 10);
			if (pAd->RaCfgObj.BssidNum > MAX_MBSSID_NUM)
			{
				pAd->RaCfgObj.BssidNum = MAX_MBSSID_NUM;
				DBGPRINT("BssidNum=%d(MAX_MBSSID_NUM is %d)\n", pAd->RaCfgObj.BssidNum, MAX_MBSSID_NUM);
			}
			else
			{
				DBGPRINT("BssidNum=%d\n", pAd->RaCfgObj.BssidNum);
			}
		}
		// VLAN_ID
		if (profile_get_keyparameter("VLAN_ID", tmpbuf, 256, profilebuf))
		{
			for (i=0, macptr = rstrtok(tmpbuf,";"); macptr; macptr = rstrtok(NULL,";"), i++)
			{
				if (i >= pAd->RaCfgObj.BssidNum)
				{
					break;
				}

				pAd->RaCfgObj.MBSSID[i].vlan_id = simple_strtol(macptr, 0, 10);
				DBGPRINT("VLAN_ID[%d]=%d\n", i, pAd->RaCfgObj.MBSSID[i].vlan_id);
			}
		}

		// VLAN_TAG
		if (profile_get_keyparameter("VLAN_TAG", tmpbuf, 256, profilebuf))
		{
			for (i=0, macptr = rstrtok(tmpbuf,";"); macptr; macptr = rstrtok(NULL,";"), i++)
			{
				if (i >= pAd->RaCfgObj.BssidNum)
				{
					break;
				}

				pAd->RaCfgObj.MBSSID[i].bVLAN_tag = (unsigned char) simple_strtol(macptr, 0, 10) > 0 ? TRUE : FALSE;
				DBGPRINT("VLAN_TAG[%d]=%d\n", i, pAd->RaCfgObj.MBSSID[i].bVLAN_tag);
			}
		}

		// WDS
		if (profile_get_keyparameter("WdsEnable", tmpbuf, 10, profilebuf))
		{

			pAd->RaCfgObj.bWds = (unsigned char) simple_strtol(tmpbuf, 0, 10) > 0 ? TRUE : FALSE;
		}

		// APCLI
		if (profile_get_keyparameter("ApCliEnable", tmpbuf, 10, profilebuf))
		{

			pAd->RaCfgObj.bApcli = (unsigned char) simple_strtol(tmpbuf, 0, 10) > 0 ? TRUE : FALSE;
		}
	}

	// Concurrent Related
	if (profile_get_keyparameter("DisableRadio", tmpbuf, 10, profilebuf))
	{
		pAd->RaCfgObj.bRadioOff = (unsigned char) simple_strtol(tmpbuf, 0, 10) > 0 ? TRUE : FALSE;
	}

	if (profile_get_keyparameter("ExtEEPROM", tmpbuf, 10, profilebuf))
	{
		pAd->RaCfgObj.bExtEEPROM = (unsigned char) simple_strtol(tmpbuf, 0, 10) > 0 ? TRUE : FALSE;
	}

#if (CONFIG_INF_TYPE==INIC_INF_TYPE_MII)
	// For locally administered address
	if (profile_get_keyparameter("LocalAdminAddr", tmpbuf, 10, profilebuf))
	{
		pAd->RaCfgObj.bLocalAdminAddr = (unsigned char) simple_strtol(tmpbuf, 0, 10) > 0 ? TRUE : FALSE;
	}
#endif
exit:
	kfree(profilebuf);
	kfree(tmpbuf);
	release_firmware(fw);
}
