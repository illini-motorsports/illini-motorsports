/**
 * Compute Header
 *
 * @author Andrew Mass
 * @date Created: 2016-03-15
 * @date Modified: 2016-03-15
 */
#ifndef COMPUTE_H
#define COMPUTE_H

#include <QThread>
#include <QObject>
#include "data.h"

/**
 * Class which handles multithreading of the conversion process so that we
 * don't lock up the GUI thread while converting data.
 */
class ComputeThread : public QThread {
  Q_OBJECT

  public:

    /**
     * Boolean which represents whether this thread is going to be used to
     * convert a Vector log file or a custom log file.
     */
    bool isVectorFile;

    /**
     * A list of names of files to convert.
     */
    QStringList filenames;

    /**
     * Pointer to the instance of the data class used for the computation.
     */
    AppData* data;

  signals:

    /**
     * Signal to be executed upon finishing the computation.
     *
     * @param success Whether the conversion was successful.
     */
    void finish(bool success);

    /**
     * Signal to be emitted whenever we need to add another file progress
     * bar to the progress bar layout.
     *
     * @param filename - The name of the file to add the progress bar for.
     */
    void addFileProgress(QString filename);

  private:

    /**
     * Starts the thread's main computation.
     */
    void run();
};

/**
 * Class with handles multithreading of the coalesce process so that we
 * don't lock up the GUI thread while coalescing logfiles.
 */
class CoalesceComputeThread : public QThread {
  Q_OBJECT

  public:

    /**
     * A list of logfiles to coalesce.
     */
    QStringList filenames;

    /**
     * Pointer to the instance of the data class used for the computation.
     */
    AppData* data;

  signals:

    /**
     * Signal to be executed upon finishing the computation.
     *
     * @param success Whether the conversion was successful.
     */
    void finish(bool success);

  private:

    /**
     * Starts the thread's main computation.
     */
    void run();
};

#endif /* COMPUTE_H */
