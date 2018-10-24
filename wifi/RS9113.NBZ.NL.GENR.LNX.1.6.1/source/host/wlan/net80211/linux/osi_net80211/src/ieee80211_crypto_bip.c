/*-
 * Copyright (c) 2002-2008 Sam Leffler, Errno Consulting
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifdef CONFIG_11W
#ifdef __FREEBSD__
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/socket.h>
#include <sys/endian.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/if_arp.h>
#include <net/if_llc.h>

#ifdef INET
#include <netinet/in.h>
#include <netinet/if_ether.h>
#endif

#include <net80211/ieee80211_priv.h>
#endif

#include <net80211/ieee80211_var.h>
#include <net80211/ieee80211_crypto.h>

#include <net80211/rijndael.h>
#include <net80211/cmac.h>

/* number of references from net80211 layer */

/* unaligned little endian access */
#define LE_WRITE_2(p, v) do {			\
	((u_int8_t *)(p))[0] = (v) & 0xff;	\
	((u_int8_t *)(p))[1] = (v) >> 8;	\
} while (0)

/* unaligned big endian access */
#define BE_READ_2(p)				\
	((u_int16_t)(p)[0] << 8 | (u_int16_t)(p)[1])

#define BE_WRITE_2(p, v) do {			\
	(p)[0] = (v) >>  8; (p)[1] = (v);	\
} while (0)

#define BE_WRITE_8(p, v) do {			\
	(p)[0] = (v) >> 56; (p)[1] = (v) >> 48;	\
	(p)[2] = (v) >> 40; (p)[3] = (v) >> 32;	\
	(p)[4] = (v) >> 24; (p)[5] = (v) >> 16;	\
	(p)[6] = (v) >>  8; (p)[7] = (v);	\
} while (0)

/* unaligned little endian access */
#define LE_WRITE_6(p, v) do {			\
	(p)[5] = (v) >> 40; (p)[4] = (v) >> 32;	\
	(p)[3] = (v) >> 24; (p)[2] = (v) >> 16;	\
	(p)[1] = (v) >>  8; (p)[0] = (v);	\
} while (0)

#define LE_READ_6(p) \
	((u_int64_t)(p)[5] << 40 | (u_int64_t)(p)[4] << 32 | \
	(u_int64_t)(p)[3] << 24 | (u_int64_t)(p)[2] << 16 | \
	(u_int64_t)(p)[1] << 8 | (u_int64_t)(p)[0])	

#define LSHIFT(v, r) do {                                       \
        int i;                                                  \
        for (i = 0; i < 15; i++)                                \
                (r)[i] = (v)[i] << 1 | (v)[i + 1] >> 7;         \
        (r)[15] = (v)[15] << 1;                                 \
} while (0)

#define XOR(v, r) do {                                          \
        int i;                                                  \
        for (i = 0; i < 16; i++)                                \
                (r)[i] ^= (v)[i];                               \
} while (0)


static void *
aes_cmac_attach(struct ieee80211vap *vap, struct ieee80211_key *k)
{
	AES_CMAC_CTX *ctx;

	ctx = (AES_CMAC_CTX *) malloc(sizeof(AES_CMAC_CTX),
			M_80211_CRYPTO, M_NOWAIT | M_ZERO);
	if (ctx == NULL) {
		vap->iv_stats.is_crypto_nomem++;
		return NULL;
	}
	ctx->cc_vap = vap;
	ctx->cc_ic = vap->iv_ic;
	return ctx;
}

static void
aes_cmac_detach(struct ieee80211_key *k)
{
	AES_CMAC_CTX *ctx = k->wk_private;

	if (ctx != NULL)
	free(ctx, M_80211_CRYPTO);
}

static int
aes_cmac_enmic(struct ieee80211_key *k, struct mbuf *m, int force)
{

	return 1;
}

static int
aes_cmac_demic(struct ieee80211_key *k, struct mbuf *m, int force)
{

	return 1;
}

void
AES_CMAC_Init(AES_CMAC_CTX *ctx)
{
        memset(ctx->X, 0, sizeof ctx->X);
        ctx->M_n = 0;
}

int
rijndael_set_key_enc_only(rijndael_ctx *ctx, const u_char *key, int bits)
{
        int rounds;

        rounds = rijndaelKeySetupEnc(ctx->ek, key, bits);
        if (rounds == 0)
                return -1;

        ctx->Nr = rounds;
//        ctx->enc_only = 1;

        return 0;
}

void
AES_CMAC_SetKey(AES_CMAC_CTX *ctx, const u_int8_t key[AES_CMAC_KEY_LENGTH])
{
        rijndael_set_key_enc_only(&ctx->rijndael, key, 128);
}

void
AES_CMAC_Update(AES_CMAC_CTX *ctx, const u_int8_t *data, u_int len)
{
        u_int mlen;

        if (ctx->M_n > 0) {
                mlen = MIN(16 - ctx->M_n, len);
                memcpy(ctx->M_last + ctx->M_n, data, mlen);
                ctx->M_n += mlen;
                if (ctx->M_n < 16 || len == mlen)
                        return;
                XOR(ctx->M_last, ctx->X);
                rijndael_encrypt(&ctx->rijndael, ctx->X, ctx->X);
                data += mlen;
                len -= mlen;
        }
        while (len > 16) {      /* not last block */
                XOR(data, ctx->X);
                rijndael_encrypt(&ctx->rijndael, ctx->X, ctx->X);
                data += 16;
                len -= 16;
        }
        /* potential last block, save it */
        memcpy(ctx->M_last, data, len);
        ctx->M_n = len;
}

void
AES_CMAC_Final(u_int8_t digest[AES_CMAC_DIGEST_LENGTH], AES_CMAC_CTX *ctx)
{
        u_int8_t K[16];

        /* generate subkey K1 */
        memset(K, 0, sizeof K);
        rijndael_encrypt(&ctx->rijndael, K, K);

        if (K[0] & 0x80) {
                LSHIFT(K, K);
                K[15] ^= 0x87;
        } else
                LSHIFT(K, K);

        if (ctx->M_n == 16) {
                /* last block was a complete block */
                XOR(K, ctx->M_last);
        } else {
                /* generate subkey K2 */
                if (K[0] & 0x80) {
                        LSHIFT(K, K);
                        K[15] ^= 0x87;
                } else
                        LSHIFT(K, K);

                /* padding(M_last) */
                ctx->M_last[ctx->M_n] = 0x80;
                while (++ctx->M_n < 16)
                        ctx->M_last[ctx->M_n] = 0;

                XOR(K, ctx->M_last);
        }
        XOR(ctx->M_last, ctx->X);
        rijndael_encrypt(&ctx->rijndael, ctx->X, digest);

        memset(K, 0, sizeof K);
}


int
timingsafe_bcmp(const void *b1, const void *b2, size_t n)
{
	const unsigned char *p1 = b1, *p2 = b2;
	int ret = 0;

	for (; n > 0; n--)
		ret |= *p1++ ^ *p2++;
	return (ret != 0);
}

/*
 *  * Initialize software crypto context.  This function can be overridden
 *   * by drivers doing hardware crypto.
 *    */
int
aes_cmac_setkey(struct ieee80211_key *k)
{
	AES_CMAC_CTX *ctx = k->wk_private;

	AES_CMAC_SetKey(ctx, k->wk_key);
	return 1;
}

/* pseudo-header used for BIP MIC computation */
struct ieee80211_bip_frame {
	u_int8_t	i_fc[2];
	u_int8_t	i_addr1[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr2[IEEE80211_ADDR_LEN];
	u_int8_t	i_addr3[IEEE80211_ADDR_LEN];
} __packed;

void dump_bip(int zone,unsigned char *vdata, int len )
{
        unsigned short ii;

        for(ii=0; ii< len; ii++)
        {
                if(!(ii % 16))
                {
                        printk("\n%04d: ", ii);
                }
                printk("%02x ",vdata[ii]);
        }
        printk("\n");
}

static int
aes_cmac_encap(struct ieee80211_key *k, struct mbuf *m0, uint8_t keyid)
{
	AES_CMAC_CTX *ctx = k->wk_private;
	struct ieee80211_bip_frame aad;
	struct ieee80211_frame *wh;
	u_int8_t mmie[IEEE80211_MMIE_LEN], mic[AES_CMAC_DIGEST_LENGTH];
	struct mbuf *m;

	wh = mtod(m0, struct ieee80211_frame *);
	KASSERT(((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
	    IEEE80211_FC0_TYPE_MGT), ("Trying to encrypt non-mgmt frames"));

	k->wk_keytsc++;
	/* clear Protected bit from group management frames */
	wh->i_fc[1] &= ~IEEE80211_FC1_PROTECTED;

	/* construct AAD (additional authenticated data) */
	aad.i_fc[0] = wh->i_fc[0];
	aad.i_fc[1] = wh->i_fc[1] & ~(IEEE80211_FC1_RETRY |
	    IEEE80211_FC1_PWR_MGT | IEEE80211_FC1_MORE_DATA);
	/* XXX 11n may require clearing the Order bit too */
	IEEE80211_ADDR_COPY(aad.i_addr1, wh->i_addr1);
	IEEE80211_ADDR_COPY(aad.i_addr2, wh->i_addr2);
	IEEE80211_ADDR_COPY(aad.i_addr3, wh->i_addr3);

	AES_CMAC_Init(ctx);
	AES_CMAC_Update(ctx, (u_int8_t *)&aad, sizeof aad);
	AES_CMAC_Update(ctx, (u_int8_t *)&wh[1],
	    m0->m_len - sizeof(*wh));

	m = m0;
	/* reserve trailing space for MMIE */
#if 0
	if (M_TRAILINGSPACE(m) < IEEE80211_MMIE_LEN) {
//		MGET(m->m_next, M_DONTWAIT, m->m_type); FIXME
		if (m->m_next == NULL)
			goto nospace;
		m = m->m_next;
		m->m_len = 0;
	}
#endif

	printk("%s %d\n",__func__,__LINE__);
	/* construct Management MIC IE */
	mmie[0] = IEEE80211_ELEMID_MMIE;
	mmie[1] = 16;
	LE_WRITE_2(&mmie[2], k->wk_keyix);
	LE_WRITE_6(&mmie[4], k->wk_keytsc);
	memset(&mmie[10], 0, 8);	/* MMIE MIC field set to 0 */

	AES_CMAC_Update(ctx, mmie, IEEE80211_MMIE_LEN);
	AES_CMAC_Final(mic, ctx);
	/* truncate AES-128-CMAC to 64-bit */
	memcpy(&mmie[10], mic, 8);

	m_append(m0, IEEE80211_MMIE_LEN, mmie);
	//m->m_len += IEEE80211_MMIE_LEN;
	//m0->m_pkthdr.len += IEEE80211_MMIE_LEN;
	k->wk_keytsc++;

	dump_bip(0, m->m_data, m->m_len);
	return 1;
}

static int
aes_cmac_decap(struct ieee80211_key *k, struct mbuf *m0, int hdrlen)
{
	AES_CMAC_CTX *ctx = k->wk_private;
	struct ieee80211_frame *wh;
	struct ieee80211_bip_frame aad;
	u_int8_t *mmie, mic0[8], mic[AES_CMAC_DIGEST_LENGTH];
	u_int64_t ipn;

	wh = mtod(m0, struct ieee80211_frame *);
	KASSERT((wh->i_fc[0] & IEEE80211_FC0_TYPE_MASK) ==
	    IEEE80211_FC0_TYPE_MGT, ("Trying to decrypt non-mgmt frames"));

	/*
 * 	 * It is assumed that management frames are contiguous and that
 * 	 	 * the mbuf length has already been checked to contain at least
 * 	 	 	 * a header and a MMIE (checked in ieee80211_decrypt()).
 * 	 	 	 	 */
	KASSERT(m0->m_len >= sizeof(*wh) + IEEE80211_MMIE_LEN, ("Unable to insert MMIE"));
	mmie = mtod(m0, u_int8_t *) + m0->m_len - IEEE80211_MMIE_LEN;

	ipn = LE_READ_6(&mmie[4]);
	if (ipn <= k->wk_keytsc) {
		/* replayed frame, discard */
	//	ic->iv_stats.is_cmac_replays++; FIXME
		return 0;
        }

	/* save and mask MMIE MIC field to 0 */
	memcpy(mic0, &mmie[10], 8);
	memset(&mmie[10], 0, 8);

	/* construct AAD (additional authenticated data) */
	aad.i_fc[0] = wh->i_fc[0];
	aad.i_fc[1] = wh->i_fc[1] & ~(IEEE80211_FC1_RETRY |
	    IEEE80211_FC1_PWR_MGT | IEEE80211_FC1_MORE_DATA);
	/* XXX 11n may require clearing the Order bit too */
	IEEE80211_ADDR_COPY(aad.i_addr1, wh->i_addr1);
	IEEE80211_ADDR_COPY(aad.i_addr2, wh->i_addr2);
	IEEE80211_ADDR_COPY(aad.i_addr3, wh->i_addr3);

	/* compute MIC */
	AES_CMAC_Init(ctx);
	AES_CMAC_Update(ctx, (u_int8_t *)&aad, sizeof(aad));
	AES_CMAC_Update(ctx, (u_int8_t *)&wh[1],
	    m0->m_len - sizeof(*wh));
	AES_CMAC_Final(mic, ctx);

	/* check that MIC matches the one in MMIE */
	if (timingsafe_bcmp(mic, mic0, 8) != 0) {
//		ic->iv_stats.is_cmac_icv_errs++; FIXME
		//m_freem(m0);
		return 0;
	}
	/*
 * 	 * There is no need to trim the MMIE from the mbuf since it is
 * 	 	 * an information element and will be ignored by upper layers.
 * 	 	 	 * We do it anyway as it is cheap to do it here and because it
 * 	 	 	 	 * may be confused with fixed fields by upper layers.
 * 	 	 	 	 	 */
	m_adj(m0, IEEE80211_MMIE_LEN);

	/* update last seen packet number (MIC is validated) */
	k->wk_keytsc = ipn;

	return 1;
}

static const struct ieee80211_cipher aes_cmac = {
	.ic_name	= "AES-CMAC",
	.ic_cipher	= IEEE80211_CIPHER_AES_CMAC,
	.ic_header	= IEEE80211_WEP_IVLEN + IEEE80211_WEP_KIDLEN,
	.ic_trailer	= IEEE80211_WEP_CRCLEN,
	.ic_miclen	= 0,
	.ic_attach	= aes_cmac_attach,
	.ic_detach	= aes_cmac_detach,
	.ic_setkey	= aes_cmac_setkey,
	.ic_encap	= aes_cmac_encap,
	.ic_decap	= aes_cmac_decap,
	.ic_enmic	= aes_cmac_enmic,
	.ic_demic	= aes_cmac_demic,
};


IEEE80211_CRYPTO_MODULE(aes_cmac, 1);
#endif
