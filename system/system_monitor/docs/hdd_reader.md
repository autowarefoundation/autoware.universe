# hdd_reader

## Name

hdd_reader - Read S.M.A.R.T. information for monitoring HDD temperature

## Synopsis

hdd_reader [OPTION]

## Description

Read S.M.A.R.T. information for monitoring HDD temperature.<br>
This runs as a daemon process and listens to a TCP/IP port (7635 by default).

**Options:**<br>
_-h, --help_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Display help<br>
_-p, --port #_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Port number to listen to

**Exit status:**<br>
Returns 0 if OK; non-zero otherwise.

## Notes

The 'hdd_reader' accesses minimal data enough to get Model number, Serial number, and HDD temperature.<br>
This is an approach to limit its functionality, however, the functionality can be expanded for further improvements and considerations in the future.<br><br>
**[ATA]**

| Purpose                     | Name                 | Length               |
| --------------------------- | -------------------- | -------------------- |
| Model number, Serial number | IDENTIFY DEVICE data | 256 words(512 bytes) |
| HDD temperature             | SMART READ DATA      | 256 words(512 bytes) |

For details please see the documents below.<br>

- [ATA Command Set - 4 (ACS-4)](http://www.t13.org/Documents/UploadedDocuments/docs2016/di529r14-ATAATAPI_Command_Set_-_4.pdf)
- [ATA/ATAPI Command Set - 3 (ACS-3)](http://www.t13.org/Documents/UploadedDocuments/docs2013/d2161r5-ATAATAPI_Command_Set_-_3.pdf)
- [SMART Attribute Overview](http://www.t13.org/Documents/UploadedDocuments/docs2005/e05171r0-ACS-SMARTAttributes_Overview.pdf)
- [SMART Attribute Annex](http://www.t13.org/documents/uploadeddocuments/docs2005/e05148r0-acs-smartattributesannex.pdf)

**[NVMe]**

| Purpose                     | Name                               | Length           |
| --------------------------- | ---------------------------------- | ---------------- |
| Model number, Serial number | Identify Controller data structure | 4096 bytes       |
| HDD temperature             | SMART / Health Information         | 1 Dword(4 bytes) |

For details please see the documents below.<br>

- [NVM Express 1.2b](https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf)

## Operation confirmed drives

- SAMSUNG MZVLB1T0HALR (SSD)
- Western Digital My Passport (Portable HDD)
