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

/*
 * A program to verify that the chunk indexing type of a dataset in a file
 * is version 1 B-tree.  
 * This is to support the testing of the tool "h5format_convert".
 */

#include "hdf5.h"
#include "H5private.h"
#include "h5tools.h"

static void usage(void);

static void
usage(void)
{
    HDfprintf(stdout, "Usage: h5fc_chk_idx file_name dataset_pathname\n");
} /* usage() */

/*-------------------------------------------------------------------------
 * Function:	main
 *
 * Purpose:	To check that the chunk indexing type for the dataset in 
 *		the file is version 1 B-tree.
 *
 * Return:	0 -- the indexing type is version 1 B-tree
 *		1 -- otherwise
 *
 *-------------------------------------------------------------------------
 */
int
main(int argc, char *argv[])
{
    char *fname = NULL;
    char *dname = NULL;
    hid_t fid = -1;
    hid_t did = -1;
    H5D_chunk_index_t idx_type;

    /* h5fc_chk_idx fname dname */
    if(argc != 3) {
	usage();
	exit(EXIT_FAILURE);
    }

    /* Duplicate the file name  & dataset name */
    fname = strdup(argv[1]);
    dname = strdup(argv[2]);

    /* Try opening the file */
    if((fid = h5tools_fopen(fname, H5F_ACC_RDONLY, H5P_DEFAULT, NULL, NULL, (size_t)0)) < 0) {
	HDfprintf(stderr, "h5fc_chk_idx: unable to open the file\n");
	return EXIT_FAILURE;
    }

    /* Open the dataset */
    if((did = H5Dopen2(fid, dname, H5P_DEFAULT)) < 0) {
	HDfprintf(stderr, "h5fc_chk_idx: unable to open the dataset\n");
	exit(EXIT_FAILURE);
    }

    /* Get the dataset's chunk indexing type */
    if(H5Dget_chunk_index_type(did, &idx_type) < 0) {
	HDfprintf(stderr, "h5fc_chk_idx: unable to get chunk index type for the dataset\n");
	exit(EXIT_FAILURE);
    }

    /* Close the dataset */
    if(H5Dclose(did) < 0) {
	HDfprintf(stderr, "h5fc_chk_idx: unable to close the dataset\n");
	exit(EXIT_FAILURE);
    }

    /* Close the file */
    if(H5Fclose(fid) < 0) {
	HDfprintf(stderr, "h5fc_chk_idx_type: cannot close the file\n");
	return EXIT_FAILURE;
    }

    /* Return success when the chunk indexing type is version 1 B-tree */
    if(idx_type == H5D_CHUNK_IDX_BTREE) 
	return(EXIT_SUCCESS);
    else {
	HDfprintf(stderr, "Error: chunk indexing type is %d\n", idx_type);
	return(EXIT_FAILURE);
    }
} /* main() */
