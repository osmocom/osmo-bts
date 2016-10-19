#!/bin/sh

# Split common DSP call log file (produced by superfemto-compatible firmware) into 4 separate call leg files (MO/MT & DL/UL) with events in format "FN EVENT_TYPE":
# MO Mobile Originated
# MT Mobile Terminated
# DL DownLink (BTS -> L1)
# UL UpLink (L1 -> BTS)

if [ -z $1 ]; then
    echo "expecting DSP log file name as parameter"
    exit 1
fi

# MO handle appear 1st in the logs
MO=$(grep 'h=' $1 | head -n 1 | cut -f2 -d',' | cut -f2 -d= | cut -f1 -d']')

# direction markers:
DLST="_CodeBurst"
ULST="_DecodeAndIdentify"

# DL sed filters:
D_EMP='s/ Empty frame request!/EMPTY/'
D_FAC='s/ Coding a FACCH\/F frame !!/FACCH/'
D_FST='s/ Coding a RTP SID First frame !!/FIRST/'
D_UPD='s/ Coding a RTP SID Update frame !!/UPDATE/'
D_SPE='s/ Coding a RTP Speech frame !!/SPEECH/'
D_ONS='s/ Coding a Onset frame !!/ONSET/'
D_FO1='s/ A speech frame is following a NoData or SID First without an Onset./FORCED_FIRST/'
D_FO2='s/ A speech frame is following a NoData without an Onset./FORCED_NODATA/'

# UL sed filters:
U_NOD='s/ It is a No Data frame !!/NODATA/'
U_ONS='s/ It is an ONSET frame !!/ONSET/'
U_UPD='s/ It is a SID UPDATE frame !!/UPDATE/'
U_FST='s/ It is a SID FIRST frame !!/FIRST/'
U_SPE='s/ It is a SPEECH frame !!/SPEECH/'

DL () { # filter downlink-related entries
    grep $DLST $1 > $1.DL.tmp
}

UL () { # uplink does not require special fix
    grep $ULST $1 > $1.UL.tmp.fix
}

DL $1
UL $1

FIX() { # add MO/MT marker from preceding line to inserted ONSETs so filtering works as expected
    cat $1.DL.tmp | awk 'BEGIN{ FS="h="; H="" } { if (NF > 1) { H = $2; print $1 "h=" $2 } else { print $1 ", h=" H } }' > $1.DL.tmp.fix
}

FIX $1

MO() { # filter MO call DL or UL logs
    grep "h=$MO" $1.tmp.fix > $1.MO.raw
}

MT() { # filter MT call DL or UL logs
    grep -v "h=$MO" $1.tmp.fix > $1.MT.raw
}

MO $1.DL
MT $1.DL
MO $1.UL
MT $1.UL

PREP() { # prepare logs for reformatting
    cat $1.raw | cut -f2 -d')' | cut -f1 -d',' | cut -f2 -d'>' | sed 's/\[u32Fn/fn/' | sed 's/fn = /fn=/' | sed 's/fn=//' | sed 's/\[Fn=//' | sed 's/ An Onset will be inserted.//' > $1.tmp1
}

PREP "$1.DL.MT"
PREP "$1.DL.MO"
PREP "$1.UL.MT"
PREP "$1.UL.MO"

RD() { # reformat DL logs for consistency checks
    cat $1.tmp1 | sed "$D_FST" | sed "$D_SPE" | sed "$D_UPD" | sed "$D_ONS" | sed "$D_EMP" | sed "$D_FAC" | sed "$D_FO1" | sed "$D_FO2" > $1.tmp2
}

RU() { # reformat UL logs for consistency checks
    cat $1.tmp1 | sed "$U_FST" | sed "$U_SPE" | sed "$U_UPD" | sed "$U_ONS" | sed "$U_NOD" > $1.tmp2
}

RD "$1.DL.MT"
RD "$1.DL.MO"
RU "$1.UL.MT"
RU "$1.UL.MO"

SW() { # swap fields
    cat $1.tmp2 | awk '{ print $2, $1 }' > $1
}

SW "$1.DL.MT"
SW "$1.DL.MO"
SW "$1.UL.MT"
SW "$1.UL.MO"

rm $1.*.tmp*
