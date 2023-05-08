# hdd_reader

## Name

hdd_reader - Read S.M.A.R.T. information to monitor HDD

## Synopsis

hdd_reader [OPTION]

## Description

Read S.M.A.R.T. information to monitor HDD such as temperature and lifetime.<br>
This runs as a daemon process and listens to an Unix domain socket (`/tmp/hdd_reader` by default).

**Options:**<br>
_-h, --help_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Display help<br>
_-s, --socket #_<br>
&nbsp;&nbsp;&nbsp;&nbsp;Path of UNIX domain socket

**Exit status:**<br>
Returns 0 if OK; non-zero otherwise.

## Notes

The 'hdd_reader' accesses minimal data enough to get Model number, Serial number, HDD temperature, and life of HDD.<br>
This is an approach to limit its functionality, however, the functionality can be expanded for further improvements and considerations in the future.<br><br>

### [ATA]

| Purpose                      | Name                 | Length               |
| ---------------------------- | -------------------- | -------------------- |
| Model number, Serial number  | IDENTIFY DEVICE data | 256 words(512 bytes) |
| HDD temperature, life of HDD | SMART READ DATA      | 256 words(512 bytes) |

For details please see the documents below.<br>
**You need to create your account of Technical Committee T13(<https://www.t13.org>).**

- [ATA Command Set - 4 (ACS-4)]
  Search document number `di529`, then you can find the document number `di529r20`.
- [ATA/ATAPI Command Set - 3 (ACS-3)]
  Search document number `d2161`, then you can find the document number `d2161r5`.
- [SMART Attribute Overview]
  Search document number `e05171`, then you can find the document number `e05171r0`.
- [SMART Attribute Annex]
  Search document number `e05148`, then you can find the document number `e05148r0`.

### [NVMe]

| Purpose                      | Name                               | Length              |
| ---------------------------- | ---------------------------------- | ------------------- |
| Model number, Serial number  | Identify Controller data structure | 4096 bytes          |
| HDD temperature, life of HDD | SMART / Health Information         | 36 Dword(144 bytes) |

For details please see the documents below.<br>

- [NVM Express 1.2b](https://www.nvmexpress.org/wp-content/uploads/NVM_Express_1_2b_Gold_20160603.pdf)

## Operation confirmed drives

<!-- cspell: ignore MZVLB1T0HALR -->

- SAMSUNG MZVLB1T0HALR (SSD)
- Western Digital My Passport (Portable HDD)
