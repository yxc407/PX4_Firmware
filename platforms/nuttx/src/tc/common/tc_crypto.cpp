/****************************************************************************
 *
 *   Copyright (c) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name TC nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#if defined(TC_CRYPTO)

#include <tc_platform_common/crypto.h>
#include <tc_platform_common/crypto_backend.h>
#include <tc_platform_common/defines.h>
#include <tc_platform/board_ctrl.h>

extern "C" {
#include <nuttx/random.h>
}


tc_sem_t TCCrypto::_lock;
bool TCCrypto::_initialized = false;

void TCCrypto::tc_crypto_init()
{
	if (TCCrypto::_initialized) {
		return;
	}

	tc_sem_init(&TCCrypto::_lock, 0, 1);

	// Initialize nuttx random pool, if it is being used by crypto
#ifdef CONFIG_CRYPTO_RANDOM_POOL
	up_randompool_initialize();
#endif

	// initialize keystore functionality
	keystore_init();

	// initialize actual crypto algoritms
	crypto_init();

	// initialize user ioctl interface for crypto
#if !defined(CONFIG_BUILD_FLAT)
	tc_register_boardct_ioctl(_CRYPTOIOCBASE, crypto_ioctl);
#endif

	TCCrypto::_initialized = true;
}

TCCrypto::TCCrypto()
{
	// Initialize an empty handle
	crypto_session_handle_init(&_crypto_handle);
}

TCCrypto::~TCCrypto()
{
	close();
}

bool TCCrypto::open(tc_crypto_algorithm_t algorithm)
{
	bool ret = false;
	lock();

	// HW specific crypto already open? Just close before proceeding
	if (crypto_session_handle_valid(_crypto_handle)) {
		crypto_close(&_crypto_handle);
	}

	// Open the HW specific crypto handle
	_crypto_handle = crypto_open(algorithm);

	if (crypto_session_handle_valid(_crypto_handle)) {
		ret = true;
	}

	unlock();

	return ret;
}

void TCCrypto::close()
{
	if (!crypto_session_handle_valid(_crypto_handle)) {
		return;
	}

	lock();
	crypto_close(&_crypto_handle);
	unlock();
}

bool TCCrypto::encrypt_data(uint8_t  key_index,
			     const uint8_t *message,
			     size_t message_size,
			     uint8_t *cipher,
			     size_t *cipher_size)
{
	return crypto_encrypt_data(_crypto_handle, key_index, message, message_size, cipher, cipher_size);
}

bool  TCCrypto::generate_key(uint8_t idx,
			      bool persistent)
{
	return crypto_generate_key(_crypto_handle, idx, persistent);
}


bool  TCCrypto::get_nonce(uint8_t *nonce,
			   size_t *nonce_len)
{
	return crypto_get_nonce(_crypto_handle, nonce, nonce_len);
}


bool  TCCrypto::get_encrypted_key(uint8_t key_idx,
				   uint8_t *key,
				   size_t *key_len,
				   uint8_t encryption_key_idx)
{
	return crypto_get_encrypted_key(_crypto_handle, key_idx, key, key_len, encryption_key_idx);
}

size_t TCCrypto::get_min_blocksize(uint8_t key_idx)
{
	return crypto_get_min_blocksize(_crypto_handle, key_idx);
}

#if !defined(CONFIG_BUILD_FLAT)
int TCCrypto::crypto_ioctl(unsigned int cmd, unsigned long arg)
{
	int ret = TC_OK;

	switch (cmd) {
	case CRYPTOIOCOPEN: {
			cryptoiocopen_t *data = (cryptoiocopen_t *)arg;
			*(data->handle) = crypto_open(data->algorithm);
		}
		break;

	case CRYPTOIOCCLOSE: {
			crypto_close((crypto_session_handle_t *)arg);
		}
		break;

	case CRYPTOIOCENCRYPT: {
			cryptoiocencrypt_t *data = (cryptoiocencrypt_t *)arg;
			data->ret = crypto_encrypt_data(*(data->handle), data->key_index, data->message, data->message_size, data->cipher,
							data->cipher_size);
		}
		break;

	case CRYPTOIOCGENKEY: {
			cryptoiocgenkey_t *data = (cryptoiocgenkey_t *)arg;
			data->ret = crypto_generate_key(*(data->handle), data->idx, data->persistent);
		}
		break;

	case CRYPTOIOCGETNONCE: {
			cryptoiocgetnonce_t *data = (cryptoiocgetnonce_t *)arg;
			data->ret = crypto_get_nonce(*(data->handle), data->nonce, data->nonce_len);

		}
		break;

	case CRYPTOIOCGETKEY: {
			cryptoiocgetkey_t *data = (cryptoiocgetkey_t *)arg;
			data->ret = crypto_get_encrypted_key(*(data->handle), data->key_idx, data->key, data->max_len,
							     data->encryption_key_idx);
		}
		break;

	case CRYPTOIOCGETBLOCKSZ: {
			cryptoiocgetblocksz_t *data = (cryptoiocgetblocksz_t *)arg;
			data->ret = crypto_get_min_blocksize(*(data->handle), data->key_idx);
		}
		break;

	default:
		ret = TC_ERROR;
		break;
	}

	return ret;
}
#endif // !defined(CONFIG_BUILD_FLAT)

#endif
