/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright by The HDF Group.                                               *
 * Copyright by the Board of Trustees of the University of Illinois.         *
 * All rights reserved.                                                      *
 *                                                                           *
 * This file is part of HDF5.  The full HDF5 copyright notice, including     *
 * terms governing use, modification, and redistribution, is contained in    *
 * the files COPYING and Copyright.html.  COPYING can be found at the root   *
 * of the source code distribution tree; Copyright.html can be found at the  *
 * root level of an installed copy of the electronic HDF5 document set and   *
 * is linked from the top-level documents page.  It can also be found at     *
 * http://hdfgroup.org/HDF5/doc/Copyright.html.  If you do not have          *
 * access to either file, you may request a copy from help@hdfgroup.org.     *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#include "hdf5.h"
#include "H5private.h"
#include "h5tools.h"

static void usage(void);

static void
usage(void)
{
    HDfprintf(stdout, "\n");
    HDfprintf(stdout, "Usage error!\n");
    HDfprintf(stdout, "Usage: clear_open_chk filename\n");
} /* usage() */

/*-------------------------------------------------------------------------
 * Function:	main
 *
 * Purpose:	To open the file which has zero or nonzero status_flags in 
 *		the superblock.
 *
 * Return:	0 on success
 *		1 on failure
 *
 * Programmer:	Vailin Choi; July 2013
 *
 *-------------------------------------------------------------------------
 */
int
main(int argc, char *argv[])
{
    char *fname;	/* The HDF5 file name */
    hid_t fid;		/* File ID */

    /* Check the # of arguments */
    if(argc != 2) {
	usage();
	return(EXIT_FAILURE);
    }

    /* Get the file name */
    fname = HDstrdup(argv[1]);

    /* Try opening the file */
    if((fid = h5tools_fopen(fname, H5F_ACC_RDONLY, H5P_DEFAULT, NULL, NULL, (size_t)0)) < 0) {
	HDfprintf(stderr, "clear_open_chk: unable to open the file\n");
	return EXIT_FAILURE;
    }

    if(fname)
        HDfree(fname);
    /* Close the file */
    if(H5Fclose(fid) < 0) {
	HDfprintf(stderr, "clear_open_chk: cannot close the file\n");
	return EXIT_FAILURE;
    }

    /* Return success */
    return EXIT_SUCCESS;

} /* main() */
