/**
 * Compute
 *
 * @author Andrew Mass
 * @date Created: 2016-03-15
 * @date Modified: 2016-03-15
 */
#include "compute.h"

void ComputeThread::run() {
  for(int i = 0; i < filenames.size(); i++) {
    this->data->filename = this->filenames.at(i);

    emit addFileProgress(this->data->filename);

    if(!(this->data->writeAxis() && this->data->readData(this->isVectorFile))) {
      finish(false);
      return;
    }
  }
  finish(true);
}

void CoalesceComputeThread::run() {
  finish(this->data->coalesceLogfiles(this->filenames));
}

