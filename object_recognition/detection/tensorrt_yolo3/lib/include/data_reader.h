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
#ifndef _DATA_READER_H_
#define _DATA_READER_H_

#include <list>
#include <string>
#include <vector>

namespace Tn
{
std::list<std::string> readFileList(const std::string & fileName);

struct Source
{
  std::string fileName;
  int label;
};
std::list<Source> readLabelFileList(const std::string & fileName);

struct Bbox
{
  int classId;
  int left;
  int right;
  int top;
  int bot;
  float score;
};
//[lst<filename>,lst<bbox_vec>]
std::tuple<std::list<std::string>, std::list<std::vector<Bbox>>> readObjectLabelFileList(
  const std::string & fileName);
}  // namespace Tn

#endif
