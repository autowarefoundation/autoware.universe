/*
 * MIT License
 * 
 * Copyright (c) 2018 lewes6369
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
#ifndef _YOLO_CONFIGS_H_
#define _YOLO_CONFIGS_H_

namespace Yolo
{
static constexpr int CHECK_COUNT = 3;
static constexpr float IGNORE_THRESH = 0.5f;
static constexpr int CLASS_NUM = 80;

struct YoloKernel
{
  int width;
  int height;
  float anchors[CHECK_COUNT * 2];
};

// YOLO 608
// YoloKernel yolo1 = {
//     19,
//     19,
//     {116,90,  156,198,  373,326}
// };
// YoloKernel yolo2 = {
//     38,
//     38,
//     {30,61,  62,45,  59,119}
// };
// YoloKernel yolo3 = {
//     76,
//     76,
//     {10,13,  16,30,  33,23}
// };

// YOLO 416
YoloKernel yolo1 = {13, 13, {116, 90, 156, 198, 373, 326}};
YoloKernel yolo2 = {26, 26, {30, 61, 62, 45, 59, 119}};
YoloKernel yolo3 = {52, 52, {10, 13, 16, 30, 33, 23}};
}  // namespace Yolo

#endif