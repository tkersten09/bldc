/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#ifndef ARRAY_H_
#define ARRAY_H_

#include <stdint.h>

// Settings
#define PACKET_RX_TIMEOUT       1000
#define PACKET_HANDLERS         2
#define PACKET_MAX_PL_LEN       1024

// Functions
void init_array(Array *a, size_t initial_size);
void insert_array(Array *a, int element);
void free_array(Array *a);

#endif /* ARRAY_H_ */
