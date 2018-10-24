#ifndef _OSI_COMMON_H
#define _OSI_COMMON_H_

#define MAC_FMT "%02x:%02x:%02x:%02x:%02x:%02x"

#define	ETHER_IS_MULTICAST(addr) (*(addr) & 0x01) /* is address mcast/bcast? */

#define	IFNAMSIZ	16

#define	IFT_ETHER	0x6		/* Ethernet CSMA/CD */
#ifdef __FREEBSD__
#define	IFT_IEEE80211	0x47 /* radio spread spectrum	*/
#elif __LINUX__
#define	IFT_IEEE80211	0x1 /* radio spread spectrum	*/
#endif

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,15,0))
#define	IFF_UP		0x1		/* (n) interface is up */
#define	IFF_BROADCAST	0x2		/* (i) broadcast address valid */
#define	IFF_PROMISC	0x100		/* (n) receive all packets */
#define	IFF_ALLMULTI	0x200		/* (n) receive all multicast packets */
#define	IFF_SIMPLEX	0x800		/* (i) can't hear own transmissions */
#endif
#define	IFF_MONITOR	0x40000		/* (n) user-requested monitor mode */

#define	IFQ_MAXLEN	50
//#define IFQ_SET_MAXLEN(ifq, len)	(ifq)->ifq_maxlen = len /* */

/*
 * Values for if_link_state.
 */
#define	LINK_STATE_UNKNOWN	0	/* link invalid/unknown */
#define	LINK_STATE_DOWN		1	/* link is down */
#define	LINK_STATE_UP		2	/* link is up */

#define LLC_UI		0x3
#define LLC_SNAP_LSAP	0xaa

#define DLT_IEEE802_11_RADIO	127
#define	NBBY	8			/* number of bits/byte */

#define	ENOENT		2		/* No such file or directory */
#define	EIO		5		/* Input/output error */
#define	ENXIO		6		/* Device not configured */
#define	EINVAL		22		/* Invalid argument */
#ifndef EOPNOTSUPP
#define	EOPNOTSUPP	95		/* Operation not supported */
#endif
#define	ENOSPC		28		/* No space left on device */
#define	EACCES		13		/* Permission denied */
#define	ENOMEM		12		/* Cannot allocate memory */
#ifndef ENETRESET
#define	ENETRESET       102		/* Network dropped connection on reset */
#endif
#ifndef ENETDOWN
#define	ENETDOWN	100		/* Network is down */
#endif
#ifndef ERESTART
#define ERESTART        85              /* restart syscall */
#endif
#define	EBUSY		16		/* Device busy */
#ifndef EADDRNOTAVAIL
#define	EADDRNOTAVAIL	99		/* Can't assign requested address */
#endif
#define	E2BIG		7		/* Argument list too long */
#ifndef ENOSYS
#define	ENOSYS		38		/* Function not implemented */
#endif
#define	EFAULT		14		/* Bad address */
#ifndef EHOSTUNREACH
#define	EHOSTUNREACH	133		/* No route to host */
#endif
#ifndef EINPROGRESS
#define	EINPROGRESS	115		/* Operation now in progress */
#endif
#define	EEXIST		17		/* File exists */
#ifndef ENOBUFS
#define ENOBUFS 	105				/* No buffer space available */
#endif
#ifndef EALREADY
#define	EALREADY	114		/* Operation already in progress */
#endif

#define	EVL_VLID_MASK		0x0FFF
#define	EVL_VLANOFTAG(tag)	((tag) & EVL_VLID_MASK)
#define	EVL_PRIOFTAG(tag)	(((tag) >> 13) & 7)

#define	IFM_TMASK	0x0000001f	/* Media sub-type */
#define	IFM_MMASK	0x00070000	/* Mode */
#define	IFM_FLAG0	0x01000000	/* Driver defined flag */
#define	IFM_AUTO	0		/* Autoselect best media */
#define	IFM_AVALID	0x00000001	/* Active bit valid */
#define	IFM_ACTIVE	0x00000002	/* Interface attached to working net */
#define	IFM_MSHIFT	16		/* Mode shift */
#define	IFM_ISHIFT	28		/* Instance shift */

/*
 * Macro to create a media word.
 */
#define	IFM_MAKEWORD(type, subtype, options, instance)			\
	((type) | (subtype) | (options) | ((instance) << IFM_ISHIFT))
#define	IFM_MAKEMODE(mode) \
	(((mode) << IFM_MSHIFT) & IFM_MMASK)

#define	IFM_SUBTYPE(x)      ((x) & IFM_TMASK)
#define	IFM_MODE(x)	    ((x) & IFM_MMASK)


#define	IFM_IEEE80211	0x00000080
#define	IFM_IEEE80211_TURBO	0x00001000	/* Operate in turbo mode */
#define	IFM_IEEE80211_ADHOC	0x00000100	/* Operate in Adhoc mode */
#define	IFM_IEEE80211_HOSTAP	0x00000200	/* Operate in Host AP mode */
#define	IFM_IEEE80211_IBSS	0x00000400	/* Operate in IBSS mode */
#define	IFM_IEEE80211_WDS		0x00000800	/* Operate in WDS mode */
#define	IFM_IEEE80211_TURBO	0x00001000	/* Operate in turbo mode */
#define	IFM_IEEE80211_MONITOR	0x00002000	/* Operate in monitor mode */
#define	IFM_IEEE80211_MBSS	0x00004000	/* Operate in MBSS mode */

#define	IFM_IEEE80211_11A	0x00010000	/* 5Ghz, OFDM mode */
#define	IFM_IEEE80211_11B	0x00020000	/* Direct Sequence mode */
#define	IFM_IEEE80211_11G	0x00030000	/* 2Ghz, CCK mode */
#define	IFM_IEEE80211_FH	0x00040000	/* 2Ghz, GFSK mode */
#define	IFM_IEEE80211_11NA	0x00050000	/* 5Ghz, HT mode */
#define	IFM_IEEE80211_11NG	0x00060000	/* 2Ghz, HT mode */
#define	IFM_AUTO			0       

#define	IFM_IEEE80211	0x00000080
/* NB: 0,1,2 are auto, manual, none defined below */
#define	IFM_IEEE80211_FH1	3	/* Frequency Hopping 1Mbps */
#define	IFM_IEEE80211_FH2	4	/* Frequency Hopping 2Mbps */
#define	IFM_IEEE80211_DS1	5	/* Direct Sequence 1Mbps */
#define	IFM_IEEE80211_DS2	6	/* Direct Sequence 2Mbps */
#define	IFM_IEEE80211_DS5	7	/* Direct Sequence 5.5Mbps */
#define	IFM_IEEE80211_DS11	8	/* Direct Sequence 11Mbps */
#define	IFM_IEEE80211_DS22	9	/* Direct Sequence 22Mbps */
#define	IFM_IEEE80211_OFDM6	10	/* OFDM 6Mbps */
#define	IFM_IEEE80211_OFDM9	11	/* OFDM 9Mbps */
#define	IFM_IEEE80211_OFDM12	12	/* OFDM 12Mbps */
#define	IFM_IEEE80211_OFDM18	13	/* OFDM 18Mbps */
#define	IFM_IEEE80211_OFDM24	14	/* OFDM 24Mbps */
#define	IFM_IEEE80211_OFDM36	15	/* OFDM 36Mbps */
#define	IFM_IEEE80211_OFDM48	16	/* OFDM 48Mbps */
#define	IFM_IEEE80211_OFDM54	17	/* OFDM 54Mbps */
#define	IFM_IEEE80211_OFDM72	18	/* OFDM 72Mbps */
#define	IFM_IEEE80211_DS354k	19	/* Direct Sequence 354Kbps */
#define	IFM_IEEE80211_DS512k	20	/* Direct Sequence 512Kbps */
#define	IFM_IEEE80211_OFDM3	21	/* OFDM 3Mbps */
#define	IFM_IEEE80211_OFDM4	22	/* OFDM 4.5Mbps */
#define	IFM_IEEE80211_OFDM27	23	/* OFDM 27Mbps */

#define	IFM_IEEE80211_MCS	24	/* HT MCS rate */


//#define PAGE_SHIFT	12  /* for 4k page_size */
//#define PAGE_SIZE	(1<<PAGE_SHIFT)	/* bytes/page */

#ifndef FALSE
#define FALSE 0
#endif
#define false FALSE

#define	PRI_MIN			(0)		/* Highest priority. */
#define	PRI_MAX			(255)		/* Lowest priority. */
#define	PRI_MIN_ITHD		(PRI_MIN)
#define	PI_NET			(PRI_MIN_ITHD + 16)

#ifndef roundup
#define	roundup(x, y)	((((x)+((y)-1))/(y))*(y))
#endif
#define	roundup2(x, y)	(((x)+((y)-1))&(~((y)-1))) /* if y is powers of two */
#define	howmany(x, y)	(((x)+((y)-1))/(y))

#define ovbcopy(f, t, l) memmove(t,f,l)
#define bcopy(a,b,c)     memcpy(b,a,c)
#define bzero(a,b)       memset(a,0,b)

#define	setbit(a,i)	(((unsigned char *)(a))[(i)/NBBY] |= 1<<((i)%NBBY))
#define	isset(a,i)							\
	(((const unsigned char *)(a))[(i)/NBBY] & (1<<((i)%NBBY)))
#define	clrbit(a,i)	(((unsigned char *)(a))[(i)/NBBY] &= ~(1<<((i)%NBBY)))
#define	isclr(a,i)							\
	((((const unsigned char *)(a))[(i)/NBBY] & (1<<((i)%NBBY))) == 0)

#if 0
This should go in the OSD file
// for _LITTLE_ENDIAN
#ifndef htole16
#define	htole16(x)	((unsigned short)(x))
#endif
#ifndef le16toh
#define	le16toh(x)	((unsigned short)(x))
#endif
#ifndef htole32
#define	htole32(x)	((uint32_t)(x))
#endif
#ifndef le64toh
#define	le64toh(x)	((uint64_t)(x))
#endif
#ifndef le32toh
#define	le32toh(x)	((uint32_t)(x))
#endif
#endif

#ifndef _ALIGNBYTES
#define	_ALIGNBYTES	(sizeof(int) - 1)
#endif
#ifndef _ALIGN
#define	_ALIGN(p)	(((unsigned long)(p) + _ALIGNBYTES) & ~_ALIGNBYTES)
#else
#undef _ALIGN
#define	_ALIGN(p)	(((unsigned long)(p) + _ALIGNBYTES) & ~_ALIGNBYTES)
#endif
#ifndef ALIGN
#define	ALIGN(p)		_ALIGN(p)
#else
#undef ALIGN
#define	ALIGN(p)		_ALIGN(p)
#endif

#define	__CONCAT2(x0,y0,x1,y1,x2,y2,x3,y3,x4,y4)	\
	x0 ## y0, ## x1 ## y1, ## x2 ## y2, ## x3 ## y4, ## x5 ## y5
#define	__CONCAT1(x,y)	x ## y
#define	__CONCAT(x,y)	__CONCAT1(x,y)
#define SET_DECLARE(set, ptype)					\
	extern ptype *__CONCAT(__start_set_,set);	\
	extern ptype *__CONCAT(__stop_set_,set)

#define SET_DECLARE_TIMEPROC(set, ptype)		\
	CALLBACK ptype *__CONCAT(__stop_set_,set)

#define SET_BEGIN(set)							\
	(&__CONCAT(__start_set_,set))
#define SET_LIMIT(set)							\
	(&__CONCAT(__stop_set_,set))


#define SET_FOREACH(pvar, set)						\
	for (pvar = SET_BEGIN(set); pvar < SET_LIMIT(set); pvar++)


#define	QMD_TRACE_ELEM(elem)
#define	QMD_TRACE_HEAD(head)
#define	QMD_SAVELINK(name, link)
#define	TRACEBUF
#define	TRASHIT(x)


#define	QMD_LIST_CHECK_HEAD(head, field)
#define	QMD_LIST_CHECK_NEXT(elm, field)
#define	QMD_LIST_CHECK_PREV(elm, field)

#define	QMD_TAILQ_CHECK_HEAD(head, field)
#define	QMD_TAILQ_CHECK_TAIL(head, headname)
#define	QMD_TAILQ_CHECK_NEXT(elm, field)
#define	QMD_TAILQ_CHECK_PREV(elm, field)


/*
 * Singly-linked List declarations.
 */
#define	SLIST_HEAD(name, type)						\
struct name {								\
	struct type *slh_first;	/* first element */			\
}

#define	SLIST_HEAD_INITIALIZER(head)					\
	{ NULL }

#define	SLIST_ENTRY(type)						\
struct {								\
	struct type *sle_next;	/* next element */			\
}

/*
 * Singly-linked List functions.
 */
#define	SLIST_EMPTY(head)	((head)->slh_first == NULL)

#define	SLIST_FIRST(head)	((head)->slh_first)

#define	SLIST_FOREACH(var, head, field)					\
	for ((var) = SLIST_FIRST((head));				\
	    (var);							\
	    (var) = SLIST_NEXT((var), field))

#define	SLIST_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = SLIST_FIRST((head));				\
	    (var) && ((tvar) = SLIST_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	SLIST_FOREACH_PREVPTR(var, varp, head, field)			\
	for ((varp) = &SLIST_FIRST((head));				\
	    ((var) = *(varp)) != NULL;					\
	    (varp) = &SLIST_NEXT((var), field))

#define	SLIST_INIT(head) do {						\
	SLIST_FIRST((head)) = NULL;					\
} while (0)

#define	SLIST_INSERT_AFTER(slistelm, elm, field) do {			\
	SLIST_NEXT((elm), field) = SLIST_NEXT((slistelm), field);	\
	SLIST_NEXT((slistelm), field) = (elm);				\
} while (0)

#define	SLIST_INSERT_HEAD(head, elm, field) do {			\
	SLIST_NEXT((elm), field) = SLIST_FIRST((head));			\
	SLIST_FIRST((head)) = (elm);					\
} while (0)

#define	SLIST_NEXT(elm, field)	((elm)->field.sle_next)

#define	SLIST_REMOVE(head, elm, type, field) do {			\
	QMD_SAVELINK(oldnext, (elm)->field.sle_next);			\
	if (SLIST_FIRST((head)) == (elm)) {				\
		SLIST_REMOVE_HEAD((head), field);			\
	}								\
	else {								\
		struct type *curelm = SLIST_FIRST((head));		\
		while (SLIST_NEXT(curelm, field) != (elm))		\
			curelm = SLIST_NEXT(curelm, field);		\
		SLIST_REMOVE_AFTER(curelm, field);			\
	}								\
	TRASHIT(*oldnext);						\
} while (0)

#define SLIST_REMOVE_AFTER(elm, field) do {				\
	SLIST_NEXT(elm, field) =					\
	    SLIST_NEXT(SLIST_NEXT(elm, field), field);			\
} while (0)

#define	SLIST_REMOVE_HEAD(head, field) do {				\
	SLIST_FIRST((head)) = SLIST_NEXT(SLIST_FIRST((head)), field);	\
} while (0)



/*
 * List declarations.
 */

#define	LIST_HEAD_DECLARE(name, type)					\
struct name {								\
	struct type *lh_first;	/* first element */			\
}

#define	LIST_HEAD_INITIALIZER(head)					\
	{ NULL }

#define	LIST_ENTRY(type)						\
struct {								\
	struct type *le_next;	/* next element */			\
	struct type **le_prev;	/* address of previous next element */	\
}

/*
 * List functions.
 */

#define	LIST_EMPTY(head)	((head)->lh_first == NULL)

#define	LIST_FIRST(head)	((head)->lh_first)

#define	LIST_FOREACH(var, head, field)					\
	for ((var) = LIST_FIRST((head));				\
	    (var);							\
	    (var) = LIST_NEXT((var), field))

#define	LIST_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = LIST_FIRST((head));				\
	    (var) && ((tvar) = LIST_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	LIST_INIT(head) do {						\
	LIST_FIRST((head)) = NULL;					\
} while (0)

#define	LIST_INSERT_AFTER(listelm, elm, field) do {			\
	QMD_LIST_CHECK_NEXT(listelm, field);				\
	if ((LIST_NEXT((elm), field) = LIST_NEXT((listelm), field)) != NULL)\
		LIST_NEXT((listelm), field)->field.le_prev =		\
		    &LIST_NEXT((elm), field);				\
	LIST_NEXT((listelm), field) = (elm);				\
	(elm)->field.le_prev = &LIST_NEXT((listelm), field);		\
} while (0)

#define	LIST_INSERT_BEFORE(listelm, elm, field) do {			\
	QMD_LIST_CHECK_PREV(listelm, field);				\
	(elm)->field.le_prev = (listelm)->field.le_prev;		\
	LIST_NEXT((elm), field) = (listelm);				\
	*(listelm)->field.le_prev = (elm);				\
	(listelm)->field.le_prev = &LIST_NEXT((elm), field);		\
} while (0)

#define	LIST_INSERT_HEAD(head, elm, field) do {				\
	QMD_LIST_CHECK_HEAD((head), field);				\
	if ((LIST_NEXT((elm), field) = LIST_FIRST((head))) != NULL)	\
		LIST_FIRST((head))->field.le_prev = &LIST_NEXT((elm), field);\
	LIST_FIRST((head)) = (elm);					\
	(elm)->field.le_prev = &LIST_FIRST((head));			\
} while (0)

#define	LIST_NEXT(elm, field)	((elm)->field.le_next)

#define	LIST_REMOVE(elm, field) do {					\
	QMD_SAVELINK(oldnext, (elm)->field.le_next);			\
	QMD_SAVELINK(oldprev, (elm)->field.le_prev);			\
	QMD_LIST_CHECK_NEXT(elm, field);				\
	QMD_LIST_CHECK_PREV(elm, field);				\
	if (LIST_NEXT((elm), field) != NULL)				\
		LIST_NEXT((elm), field)->field.le_prev = 		\
		    (elm)->field.le_prev;				\
	*(elm)->field.le_prev = LIST_NEXT((elm), field);		\
	TRASHIT(*oldnext);						\
	TRASHIT(*oldprev);						\
} while (0)

#define LIST_SWAP(head1, head2, type, field) do {			\
	struct type *swap_tmp = LIST_FIRST((head1));			\
	LIST_FIRST((head1)) = LIST_FIRST((head2));			\
	LIST_FIRST((head2)) = swap_tmp;					\
	if ((swap_tmp = LIST_FIRST((head1))) != NULL)			\
		swap_tmp->field.le_prev = &LIST_FIRST((head1));		\
	if ((swap_tmp = LIST_FIRST((head2))) != NULL)			\
		swap_tmp->field.le_prev = &LIST_FIRST((head2));		\
} while (0)


/*
 * Tail queue declarations.
 */

#define	TAILQ_HEAD(name, type)						\
struct name {								\
	struct type *tqh_first; /* first element */ 		\
	struct type **tqh_last; /* addr of last next element */ 	\
	TRACEBUF							\
}

#define	TAILQ_HEAD_INITIALIZER(head)					\
	{ NULL, &(head).tqh_first }

#define	TAILQ_ENTRY(type)						\
struct {								\
	struct type *tqe_next;	/* next element */			\
	struct type **tqe_prev; /* address of previous next element */	\
	TRACEBUF							\
}

#define	TAILQ_CONCAT(head1, head2, field) do {				\
	if (!TAILQ_EMPTY(head2)) {					\
		*(head1)->tqh_last = (head2)->tqh_first;		\
		(head2)->tqh_first->field.tqe_prev = (head1)->tqh_last;	\
		(head1)->tqh_last = (head2)->tqh_last;			\
		TAILQ_INIT((head2));					\
		QMD_TRACE_HEAD(head1);					\
		QMD_TRACE_HEAD(head2);					\
	}								\
} while (0)

#define	TAILQ_EMPTY(head)	((head)->tqh_first == NULL)

#define	TAILQ_FIRST(head)	((head)->tqh_first)

#define	TAILQ_FOREACH(var, head, field)					\
	for ((var) = TAILQ_FIRST((head));				\
	    (var);							\
	    (var) = TAILQ_NEXT((var), field))

#define	TAILQ_FOREACH_SAFE(var, head, field, tvar)			\
	for ((var) = TAILQ_FIRST((head));				\
	    (var) && ((tvar) = TAILQ_NEXT((var), field), 1);		\
	    (var) = (tvar))

#define	TAILQ_FOREACH_REVERSE(var, head, headname, field)		\
	for ((var) = TAILQ_LAST((head), headname);			\
	    (var);							\
	    (var) = TAILQ_PREV((var), headname, field))

#define	TAILQ_FOREACH_REVERSE_SAFE(var, head, headname, field, tvar)	\
	for ((var) = TAILQ_LAST((head), headname);			\
	    (var) && ((tvar) = TAILQ_PREV((var), headname, field), 1);	\
	    (var) = (tvar))

#define	TAILQ_INIT(head) do {						\
	TAILQ_FIRST((head)) = NULL;					\
	(head)->tqh_last = &TAILQ_FIRST((head));			\
	QMD_TRACE_HEAD(head);						\
} while (0)

#define	TAILQ_INSERT_AFTER(head, listelm, elm, field) do {		\
	QMD_TAILQ_CHECK_NEXT(listelm, field);				\
	if ((TAILQ_NEXT((elm), field) = TAILQ_NEXT((listelm), field)) != NULL)\
		TAILQ_NEXT((elm), field)->field.tqe_prev = 		\
		    &TAILQ_NEXT((elm), field);				\
	else {								\
		(head)->tqh_last = &TAILQ_NEXT((elm), field);		\
		QMD_TRACE_HEAD(head);					\
	}								\
	TAILQ_NEXT((listelm), field) = (elm);				\
	(elm)->field.tqe_prev = &TAILQ_NEXT((listelm), field);		\
	QMD_TRACE_ELEM(&(elm)->field);					\
	QMD_TRACE_ELEM(&listelm->field);				\
} while (0)

#define	TAILQ_INSERT_BEFORE(listelm, elm, field) do {			\
	QMD_TAILQ_CHECK_PREV(listelm, field);				\
	(elm)->field.tqe_prev = (listelm)->field.tqe_prev;		\
	TAILQ_NEXT((elm), field) = (listelm);				\
	*(listelm)->field.tqe_prev = (elm);				\
	(listelm)->field.tqe_prev = &TAILQ_NEXT((elm), field);		\
	QMD_TRACE_ELEM(&(elm)->field);					\
	QMD_TRACE_ELEM(&listelm->field);				\
} while (0)

#define	TAILQ_INSERT_HEAD(head, elm, field) do {			\
	QMD_TAILQ_CHECK_HEAD(head, field);				\
	if ((TAILQ_NEXT((elm), field) = TAILQ_FIRST((head))) != NULL)	\
		TAILQ_FIRST((head))->field.tqe_prev =			\
		    &TAILQ_NEXT((elm), field);				\
	else								\
		(head)->tqh_last = &TAILQ_NEXT((elm), field);		\
	TAILQ_FIRST((head)) = (elm);					\
	(elm)->field.tqe_prev = &TAILQ_FIRST((head));			\
	QMD_TRACE_HEAD(head);						\
	QMD_TRACE_ELEM(&(elm)->field);					\
} while (0)

#define	TAILQ_INSERT_TAIL(head, elm, field) do {			\
	QMD_TAILQ_CHECK_TAIL(head, field);				\
	TAILQ_NEXT((elm), field) = NULL;				\
	(elm)->field.tqe_prev = (head)->tqh_last;			\
	*(head)->tqh_last = (elm);					\
	(head)->tqh_last = &TAILQ_NEXT((elm), field);			\
	QMD_TRACE_HEAD(head);						\
	QMD_TRACE_ELEM(&(elm)->field);					\
} while (0)

#define	TAILQ_LAST(head, headname)					\
	(*(((struct headname *)((head)->tqh_last))->tqh_last))

#define	TAILQ_NEXT(elm, field) ((elm)->field.tqe_next)

#define	TAILQ_PREV(elm, headname, field)				\
	(*(((struct headname *)((elm)->field.tqe_prev))->tqh_last))

#define	TAILQ_REMOVE(head, elm, field) do {				\
	QMD_SAVELINK(oldnext, (elm)->field.tqe_next);			\
	QMD_SAVELINK(oldprev, (elm)->field.tqe_prev);			\
	QMD_TAILQ_CHECK_NEXT(elm, field);				\
	QMD_TAILQ_CHECK_PREV(elm, field);				\
	if ((TAILQ_NEXT((elm), field)) != NULL)				\
		TAILQ_NEXT((elm), field)->field.tqe_prev = 		\
		    (elm)->field.tqe_prev;				\
	else {								\
		(head)->tqh_last = (elm)->field.tqe_prev;		\
		QMD_TRACE_HEAD(head);					\
	}								\
	*(elm)->field.tqe_prev = TAILQ_NEXT((elm), field);		\
	TRASHIT(*oldnext);						\
	TRASHIT(*oldprev);						\
	QMD_TRACE_ELEM(&(elm)->field);					\
} while (0)

#define TAILQ_SWAP(head1, head2, type, field) do {			\
	struct type *swap_first = (head1)->tqh_first;			\
	struct type **swap_last = (head1)->tqh_last;			\
	(head1)->tqh_first = (head2)->tqh_first;			\
	(head1)->tqh_last = (head2)->tqh_last;				\
	(head2)->tqh_first = swap_first;				\
	(head2)->tqh_last = swap_last;					\
	if ((swap_first = (head1)->tqh_first) != NULL)			\
		swap_first->field.tqe_prev = &(head1)->tqh_first;	\
	else								\
		(head1)->tqh_last = &(head1)->tqh_first;		\
	if ((swap_first = (head2)->tqh_first) != NULL)			\
		swap_first->field.tqe_prev = &(head2)->tqh_first;	\
	else								\
		(head2)->tqh_last = &(head2)->tqh_first;		\
} while (0)

#if 0
struct route {
 int reserved[0];
};
#endif

/* Macros for min/max. */
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define SI_SUB_DRIVERS 	1	
#define SI_ORDER_FIRST	2
#define wlan_phy	3
/* rx path usage */
#define M_AMPDU         M_PROTO1                /* A-MPDU subframe */
#define M_WEP           M_PROTO2                /* WEP done by hardware */
#if 0
#define M_AMPDU_MPDU    M_PROTO8                /* A-MPDU re-order done */
#endif
#define M_80211_RX      (M_AMPDU|M_WEP|M_AMPDU_MPDU)
#define M_VLANTAG       0x00010000 /* ether_vtag is valid */

#define ETHERTYPE_AARP          0x80F3  /* AppleTalk AARP */
#define ETHERTYPE_IPX           0x8137  /* Novell (old) NetWare IPX (ECONFIG E option) */

#define __unused
#endif /* _OSI_NET80211_H_ */

