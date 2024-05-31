/* This file was automatically generated by CasADi 3.6.5+.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

int J_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int J_v_func_alloc_mem(void);
int J_v_func_init_mem(int mem);
void J_v_func_free_mem(int mem);
int J_v_func_checkout(void);
void J_v_func_release(int mem);
void J_v_func_incref(void);
void J_v_func_decref(void);
casadi_int J_v_func_n_in(void);
casadi_int J_v_func_n_out(void);
casadi_real J_v_func_default_in(casadi_int i);
const char* J_v_func_name_in(casadi_int i);
const char* J_v_func_name_out(casadi_int i);
const casadi_int* J_v_func_sparsity_in(casadi_int i);
const casadi_int* J_v_func_sparsity_out(casadi_int i);
int J_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int J_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define J_v_func_SZ_ARG 3
#define J_v_func_SZ_RES 1
#define J_v_func_SZ_IW 0
#define J_v_func_SZ_W 9
int J_vv_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int J_vv_func_alloc_mem(void);
int J_vv_func_init_mem(int mem);
void J_vv_func_free_mem(int mem);
int J_vv_func_checkout(void);
void J_vv_func_release(int mem);
void J_vv_func_incref(void);
void J_vv_func_decref(void);
casadi_int J_vv_func_n_in(void);
casadi_int J_vv_func_n_out(void);
casadi_real J_vv_func_default_in(casadi_int i);
const char* J_vv_func_name_in(casadi_int i);
const char* J_vv_func_name_out(casadi_int i);
const casadi_int* J_vv_func_sparsity_in(casadi_int i);
const casadi_int* J_vv_func_sparsity_out(casadi_int i);
int J_vv_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int J_vv_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define J_vv_func_SZ_ARG 3
#define J_vv_func_SZ_RES 1
#define J_vv_func_SZ_IW 0
#define J_vv_func_SZ_W 5
int ceq1_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int ceq1_func_alloc_mem(void);
int ceq1_func_init_mem(int mem);
void ceq1_func_free_mem(int mem);
int ceq1_func_checkout(void);
void ceq1_func_release(int mem);
void ceq1_func_incref(void);
void ceq1_func_decref(void);
casadi_int ceq1_func_n_in(void);
casadi_int ceq1_func_n_out(void);
casadi_real ceq1_func_default_in(casadi_int i);
const char* ceq1_func_name_in(casadi_int i);
const char* ceq1_func_name_out(casadi_int i);
const casadi_int* ceq1_func_sparsity_in(casadi_int i);
const casadi_int* ceq1_func_sparsity_out(casadi_int i);
int ceq1_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int ceq1_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define ceq1_func_SZ_ARG 8
#define ceq1_func_SZ_RES 1
#define ceq1_func_SZ_IW 0
#define ceq1_func_SZ_W 18
int ceq1_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int ceq1_v_func_alloc_mem(void);
int ceq1_v_func_init_mem(int mem);
void ceq1_v_func_free_mem(int mem);
int ceq1_v_func_checkout(void);
void ceq1_v_func_release(int mem);
void ceq1_v_func_incref(void);
void ceq1_v_func_decref(void);
casadi_int ceq1_v_func_n_in(void);
casadi_int ceq1_v_func_n_out(void);
casadi_real ceq1_v_func_default_in(casadi_int i);
const char* ceq1_v_func_name_in(casadi_int i);
const char* ceq1_v_func_name_out(casadi_int i);
const casadi_int* ceq1_v_func_sparsity_in(casadi_int i);
const casadi_int* ceq1_v_func_sparsity_out(casadi_int i);
int ceq1_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int ceq1_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define ceq1_v_func_SZ_ARG 8
#define ceq1_v_func_SZ_RES 1
#define ceq1_v_func_SZ_IW 0
#define ceq1_v_func_SZ_W 27
int ceq2_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int ceq2_func_alloc_mem(void);
int ceq2_func_init_mem(int mem);
void ceq2_func_free_mem(int mem);
int ceq2_func_checkout(void);
void ceq2_func_release(int mem);
void ceq2_func_incref(void);
void ceq2_func_decref(void);
casadi_int ceq2_func_n_in(void);
casadi_int ceq2_func_n_out(void);
casadi_real ceq2_func_default_in(casadi_int i);
const char* ceq2_func_name_in(casadi_int i);
const char* ceq2_func_name_out(casadi_int i);
const casadi_int* ceq2_func_sparsity_in(casadi_int i);
const casadi_int* ceq2_func_sparsity_out(casadi_int i);
int ceq2_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int ceq2_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define ceq2_func_SZ_ARG 1
#define ceq2_func_SZ_RES 1
#define ceq2_func_SZ_IW 0
#define ceq2_func_SZ_W 2
int ceq2_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int ceq2_v_func_alloc_mem(void);
int ceq2_v_func_init_mem(int mem);
void ceq2_v_func_free_mem(int mem);
int ceq2_v_func_checkout(void);
void ceq2_v_func_release(int mem);
void ceq2_v_func_incref(void);
void ceq2_v_func_decref(void);
casadi_int ceq2_v_func_n_in(void);
casadi_int ceq2_v_func_n_out(void);
casadi_real ceq2_v_func_default_in(casadi_int i);
const char* ceq2_v_func_name_in(casadi_int i);
const char* ceq2_v_func_name_out(casadi_int i);
const casadi_int* ceq2_v_func_sparsity_in(casadi_int i);
const casadi_int* ceq2_v_func_sparsity_out(casadi_int i);
int ceq2_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int ceq2_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define ceq2_v_func_SZ_ARG 1
#define ceq2_v_func_SZ_RES 1
#define ceq2_v_func_SZ_IW 0
#define ceq2_v_func_SZ_W 2
int cineq1_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq1_max_func_alloc_mem(void);
int cineq1_max_func_init_mem(int mem);
void cineq1_max_func_free_mem(int mem);
int cineq1_max_func_checkout(void);
void cineq1_max_func_release(int mem);
void cineq1_max_func_incref(void);
void cineq1_max_func_decref(void);
casadi_int cineq1_max_func_n_in(void);
casadi_int cineq1_max_func_n_out(void);
casadi_real cineq1_max_func_default_in(casadi_int i);
const char* cineq1_max_func_name_in(casadi_int i);
const char* cineq1_max_func_name_out(casadi_int i);
const casadi_int* cineq1_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq1_max_func_sparsity_out(casadi_int i);
int cineq1_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq1_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq1_max_func_SZ_ARG 3
#define cineq1_max_func_SZ_RES 1
#define cineq1_max_func_SZ_IW 0
#define cineq1_max_func_SZ_W 3
int cineq1_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq1_min_func_alloc_mem(void);
int cineq1_min_func_init_mem(int mem);
void cineq1_min_func_free_mem(int mem);
int cineq1_min_func_checkout(void);
void cineq1_min_func_release(int mem);
void cineq1_min_func_incref(void);
void cineq1_min_func_decref(void);
casadi_int cineq1_min_func_n_in(void);
casadi_int cineq1_min_func_n_out(void);
casadi_real cineq1_min_func_default_in(casadi_int i);
const char* cineq1_min_func_name_in(casadi_int i);
const char* cineq1_min_func_name_out(casadi_int i);
const casadi_int* cineq1_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq1_min_func_sparsity_out(casadi_int i);
int cineq1_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq1_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq1_min_func_SZ_ARG 3
#define cineq1_min_func_SZ_RES 1
#define cineq1_min_func_SZ_IW 0
#define cineq1_min_func_SZ_W 3
int cineq2_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq2_max_func_alloc_mem(void);
int cineq2_max_func_init_mem(int mem);
void cineq2_max_func_free_mem(int mem);
int cineq2_max_func_checkout(void);
void cineq2_max_func_release(int mem);
void cineq2_max_func_incref(void);
void cineq2_max_func_decref(void);
casadi_int cineq2_max_func_n_in(void);
casadi_int cineq2_max_func_n_out(void);
casadi_real cineq2_max_func_default_in(casadi_int i);
const char* cineq2_max_func_name_in(casadi_int i);
const char* cineq2_max_func_name_out(casadi_int i);
const casadi_int* cineq2_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq2_max_func_sparsity_out(casadi_int i);
int cineq2_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq2_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq2_max_func_SZ_ARG 3
#define cineq2_max_func_SZ_RES 1
#define cineq2_max_func_SZ_IW 0
#define cineq2_max_func_SZ_W 3
int cineq2_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq2_min_func_alloc_mem(void);
int cineq2_min_func_init_mem(int mem);
void cineq2_min_func_free_mem(int mem);
int cineq2_min_func_checkout(void);
void cineq2_min_func_release(int mem);
void cineq2_min_func_incref(void);
void cineq2_min_func_decref(void);
casadi_int cineq2_min_func_n_in(void);
casadi_int cineq2_min_func_n_out(void);
casadi_real cineq2_min_func_default_in(casadi_int i);
const char* cineq2_min_func_name_in(casadi_int i);
const char* cineq2_min_func_name_out(casadi_int i);
const casadi_int* cineq2_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq2_min_func_sparsity_out(casadi_int i);
int cineq2_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq2_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq2_min_func_SZ_ARG 3
#define cineq2_min_func_SZ_RES 1
#define cineq2_min_func_SZ_IW 0
#define cineq2_min_func_SZ_W 3
int cineq3_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq3_max_func_alloc_mem(void);
int cineq3_max_func_init_mem(int mem);
void cineq3_max_func_free_mem(int mem);
int cineq3_max_func_checkout(void);
void cineq3_max_func_release(int mem);
void cineq3_max_func_incref(void);
void cineq3_max_func_decref(void);
casadi_int cineq3_max_func_n_in(void);
casadi_int cineq3_max_func_n_out(void);
casadi_real cineq3_max_func_default_in(casadi_int i);
const char* cineq3_max_func_name_in(casadi_int i);
const char* cineq3_max_func_name_out(casadi_int i);
const casadi_int* cineq3_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq3_max_func_sparsity_out(casadi_int i);
int cineq3_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq3_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq3_max_func_SZ_ARG 3
#define cineq3_max_func_SZ_RES 1
#define cineq3_max_func_SZ_IW 0
#define cineq3_max_func_SZ_W 3
int cineq3_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq3_min_func_alloc_mem(void);
int cineq3_min_func_init_mem(int mem);
void cineq3_min_func_free_mem(int mem);
int cineq3_min_func_checkout(void);
void cineq3_min_func_release(int mem);
void cineq3_min_func_incref(void);
void cineq3_min_func_decref(void);
casadi_int cineq3_min_func_n_in(void);
casadi_int cineq3_min_func_n_out(void);
casadi_real cineq3_min_func_default_in(casadi_int i);
const char* cineq3_min_func_name_in(casadi_int i);
const char* cineq3_min_func_name_out(casadi_int i);
const casadi_int* cineq3_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq3_min_func_sparsity_out(casadi_int i);
int cineq3_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq3_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq3_min_func_SZ_ARG 3
#define cineq3_min_func_SZ_RES 1
#define cineq3_min_func_SZ_IW 0
#define cineq3_min_func_SZ_W 3
int cineq4_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq4_max_func_alloc_mem(void);
int cineq4_max_func_init_mem(int mem);
void cineq4_max_func_free_mem(int mem);
int cineq4_max_func_checkout(void);
void cineq4_max_func_release(int mem);
void cineq4_max_func_incref(void);
void cineq4_max_func_decref(void);
casadi_int cineq4_max_func_n_in(void);
casadi_int cineq4_max_func_n_out(void);
casadi_real cineq4_max_func_default_in(casadi_int i);
const char* cineq4_max_func_name_in(casadi_int i);
const char* cineq4_max_func_name_out(casadi_int i);
const casadi_int* cineq4_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq4_max_func_sparsity_out(casadi_int i);
int cineq4_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq4_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq4_max_func_SZ_ARG 2
#define cineq4_max_func_SZ_RES 1
#define cineq4_max_func_SZ_IW 0
#define cineq4_max_func_SZ_W 2
int cineq4_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq4_min_func_alloc_mem(void);
int cineq4_min_func_init_mem(int mem);
void cineq4_min_func_free_mem(int mem);
int cineq4_min_func_checkout(void);
void cineq4_min_func_release(int mem);
void cineq4_min_func_incref(void);
void cineq4_min_func_decref(void);
casadi_int cineq4_min_func_n_in(void);
casadi_int cineq4_min_func_n_out(void);
casadi_real cineq4_min_func_default_in(casadi_int i);
const char* cineq4_min_func_name_in(casadi_int i);
const char* cineq4_min_func_name_out(casadi_int i);
const casadi_int* cineq4_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq4_min_func_sparsity_out(casadi_int i);
int cineq4_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq4_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq4_min_func_SZ_ARG 2
#define cineq4_min_func_SZ_RES 1
#define cineq4_min_func_SZ_IW 0
#define cineq4_min_func_SZ_W 2
int cineq5_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq5_max_func_alloc_mem(void);
int cineq5_max_func_init_mem(int mem);
void cineq5_max_func_free_mem(int mem);
int cineq5_max_func_checkout(void);
void cineq5_max_func_release(int mem);
void cineq5_max_func_incref(void);
void cineq5_max_func_decref(void);
casadi_int cineq5_max_func_n_in(void);
casadi_int cineq5_max_func_n_out(void);
casadi_real cineq5_max_func_default_in(casadi_int i);
const char* cineq5_max_func_name_in(casadi_int i);
const char* cineq5_max_func_name_out(casadi_int i);
const casadi_int* cineq5_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq5_max_func_sparsity_out(casadi_int i);
int cineq5_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq5_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq5_max_func_SZ_ARG 2
#define cineq5_max_func_SZ_RES 1
#define cineq5_max_func_SZ_IW 0
#define cineq5_max_func_SZ_W 2
int cineq5_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq5_min_func_alloc_mem(void);
int cineq5_min_func_init_mem(int mem);
void cineq5_min_func_free_mem(int mem);
int cineq5_min_func_checkout(void);
void cineq5_min_func_release(int mem);
void cineq5_min_func_incref(void);
void cineq5_min_func_decref(void);
casadi_int cineq5_min_func_n_in(void);
casadi_int cineq5_min_func_n_out(void);
casadi_real cineq5_min_func_default_in(casadi_int i);
const char* cineq5_min_func_name_in(casadi_int i);
const char* cineq5_min_func_name_out(casadi_int i);
const casadi_int* cineq5_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq5_min_func_sparsity_out(casadi_int i);
int cineq5_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq5_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq5_min_func_SZ_ARG 2
#define cineq5_min_func_SZ_RES 1
#define cineq5_min_func_SZ_IW 0
#define cineq5_min_func_SZ_W 2
int cineq6_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq6_max_func_alloc_mem(void);
int cineq6_max_func_init_mem(int mem);
void cineq6_max_func_free_mem(int mem);
int cineq6_max_func_checkout(void);
void cineq6_max_func_release(int mem);
void cineq6_max_func_incref(void);
void cineq6_max_func_decref(void);
casadi_int cineq6_max_func_n_in(void);
casadi_int cineq6_max_func_n_out(void);
casadi_real cineq6_max_func_default_in(casadi_int i);
const char* cineq6_max_func_name_in(casadi_int i);
const char* cineq6_max_func_name_out(casadi_int i);
const casadi_int* cineq6_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq6_max_func_sparsity_out(casadi_int i);
int cineq6_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq6_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq6_max_func_SZ_ARG 6
#define cineq6_max_func_SZ_RES 1
#define cineq6_max_func_SZ_IW 0
#define cineq6_max_func_SZ_W 3
int cineq6_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq6_min_func_alloc_mem(void);
int cineq6_min_func_init_mem(int mem);
void cineq6_min_func_free_mem(int mem);
int cineq6_min_func_checkout(void);
void cineq6_min_func_release(int mem);
void cineq6_min_func_incref(void);
void cineq6_min_func_decref(void);
casadi_int cineq6_min_func_n_in(void);
casadi_int cineq6_min_func_n_out(void);
casadi_real cineq6_min_func_default_in(casadi_int i);
const char* cineq6_min_func_name_in(casadi_int i);
const char* cineq6_min_func_name_out(casadi_int i);
const casadi_int* cineq6_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq6_min_func_sparsity_out(casadi_int i);
int cineq6_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq6_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq6_min_func_SZ_ARG 6
#define cineq6_min_func_SZ_RES 1
#define cineq6_min_func_SZ_IW 0
#define cineq6_min_func_SZ_W 4
int cineq7_max_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq7_max_func_alloc_mem(void);
int cineq7_max_func_init_mem(int mem);
void cineq7_max_func_free_mem(int mem);
int cineq7_max_func_checkout(void);
void cineq7_max_func_release(int mem);
void cineq7_max_func_incref(void);
void cineq7_max_func_decref(void);
casadi_int cineq7_max_func_n_in(void);
casadi_int cineq7_max_func_n_out(void);
casadi_real cineq7_max_func_default_in(casadi_int i);
const char* cineq7_max_func_name_in(casadi_int i);
const char* cineq7_max_func_name_out(casadi_int i);
const casadi_int* cineq7_max_func_sparsity_in(casadi_int i);
const casadi_int* cineq7_max_func_sparsity_out(casadi_int i);
int cineq7_max_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq7_max_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq7_max_func_SZ_ARG 5
#define cineq7_max_func_SZ_RES 1
#define cineq7_max_func_SZ_IW 0
#define cineq7_max_func_SZ_W 8
int cineq7_min_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq7_min_func_alloc_mem(void);
int cineq7_min_func_init_mem(int mem);
void cineq7_min_func_free_mem(int mem);
int cineq7_min_func_checkout(void);
void cineq7_min_func_release(int mem);
void cineq7_min_func_incref(void);
void cineq7_min_func_decref(void);
casadi_int cineq7_min_func_n_in(void);
casadi_int cineq7_min_func_n_out(void);
casadi_real cineq7_min_func_default_in(casadi_int i);
const char* cineq7_min_func_name_in(casadi_int i);
const char* cineq7_min_func_name_out(casadi_int i);
const casadi_int* cineq7_min_func_sparsity_in(casadi_int i);
const casadi_int* cineq7_min_func_sparsity_out(casadi_int i);
int cineq7_min_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq7_min_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq7_min_func_SZ_ARG 5
#define cineq7_min_func_SZ_RES 1
#define cineq7_min_func_SZ_IW 0
#define cineq7_min_func_SZ_W 8
int cineq1_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq1_max_v_func_alloc_mem(void);
int cineq1_max_v_func_init_mem(int mem);
void cineq1_max_v_func_free_mem(int mem);
int cineq1_max_v_func_checkout(void);
void cineq1_max_v_func_release(int mem);
void cineq1_max_v_func_incref(void);
void cineq1_max_v_func_decref(void);
casadi_int cineq1_max_v_func_n_in(void);
casadi_int cineq1_max_v_func_n_out(void);
casadi_real cineq1_max_v_func_default_in(casadi_int i);
const char* cineq1_max_v_func_name_in(casadi_int i);
const char* cineq1_max_v_func_name_out(casadi_int i);
const casadi_int* cineq1_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq1_max_v_func_sparsity_out(casadi_int i);
int cineq1_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq1_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq1_max_v_func_SZ_ARG 3
#define cineq1_max_v_func_SZ_RES 1
#define cineq1_max_v_func_SZ_IW 0
#define cineq1_max_v_func_SZ_W 1
int cineq1_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq1_min_v_func_alloc_mem(void);
int cineq1_min_v_func_init_mem(int mem);
void cineq1_min_v_func_free_mem(int mem);
int cineq1_min_v_func_checkout(void);
void cineq1_min_v_func_release(int mem);
void cineq1_min_v_func_incref(void);
void cineq1_min_v_func_decref(void);
casadi_int cineq1_min_v_func_n_in(void);
casadi_int cineq1_min_v_func_n_out(void);
casadi_real cineq1_min_v_func_default_in(casadi_int i);
const char* cineq1_min_v_func_name_in(casadi_int i);
const char* cineq1_min_v_func_name_out(casadi_int i);
const casadi_int* cineq1_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq1_min_v_func_sparsity_out(casadi_int i);
int cineq1_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq1_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq1_min_v_func_SZ_ARG 3
#define cineq1_min_v_func_SZ_RES 1
#define cineq1_min_v_func_SZ_IW 0
#define cineq1_min_v_func_SZ_W 1
int cineq2_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq2_max_v_func_alloc_mem(void);
int cineq2_max_v_func_init_mem(int mem);
void cineq2_max_v_func_free_mem(int mem);
int cineq2_max_v_func_checkout(void);
void cineq2_max_v_func_release(int mem);
void cineq2_max_v_func_incref(void);
void cineq2_max_v_func_decref(void);
casadi_int cineq2_max_v_func_n_in(void);
casadi_int cineq2_max_v_func_n_out(void);
casadi_real cineq2_max_v_func_default_in(casadi_int i);
const char* cineq2_max_v_func_name_in(casadi_int i);
const char* cineq2_max_v_func_name_out(casadi_int i);
const casadi_int* cineq2_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq2_max_v_func_sparsity_out(casadi_int i);
int cineq2_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq2_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq2_max_v_func_SZ_ARG 3
#define cineq2_max_v_func_SZ_RES 1
#define cineq2_max_v_func_SZ_IW 0
#define cineq2_max_v_func_SZ_W 1
int cineq2_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq2_min_v_func_alloc_mem(void);
int cineq2_min_v_func_init_mem(int mem);
void cineq2_min_v_func_free_mem(int mem);
int cineq2_min_v_func_checkout(void);
void cineq2_min_v_func_release(int mem);
void cineq2_min_v_func_incref(void);
void cineq2_min_v_func_decref(void);
casadi_int cineq2_min_v_func_n_in(void);
casadi_int cineq2_min_v_func_n_out(void);
casadi_real cineq2_min_v_func_default_in(casadi_int i);
const char* cineq2_min_v_func_name_in(casadi_int i);
const char* cineq2_min_v_func_name_out(casadi_int i);
const casadi_int* cineq2_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq2_min_v_func_sparsity_out(casadi_int i);
int cineq2_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq2_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq2_min_v_func_SZ_ARG 3
#define cineq2_min_v_func_SZ_RES 1
#define cineq2_min_v_func_SZ_IW 0
#define cineq2_min_v_func_SZ_W 1
int cineq3_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq3_max_v_func_alloc_mem(void);
int cineq3_max_v_func_init_mem(int mem);
void cineq3_max_v_func_free_mem(int mem);
int cineq3_max_v_func_checkout(void);
void cineq3_max_v_func_release(int mem);
void cineq3_max_v_func_incref(void);
void cineq3_max_v_func_decref(void);
casadi_int cineq3_max_v_func_n_in(void);
casadi_int cineq3_max_v_func_n_out(void);
casadi_real cineq3_max_v_func_default_in(casadi_int i);
const char* cineq3_max_v_func_name_in(casadi_int i);
const char* cineq3_max_v_func_name_out(casadi_int i);
const casadi_int* cineq3_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq3_max_v_func_sparsity_out(casadi_int i);
int cineq3_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq3_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq3_max_v_func_SZ_ARG 3
#define cineq3_max_v_func_SZ_RES 1
#define cineq3_max_v_func_SZ_IW 0
#define cineq3_max_v_func_SZ_W 1
int cineq3_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq3_min_v_func_alloc_mem(void);
int cineq3_min_v_func_init_mem(int mem);
void cineq3_min_v_func_free_mem(int mem);
int cineq3_min_v_func_checkout(void);
void cineq3_min_v_func_release(int mem);
void cineq3_min_v_func_incref(void);
void cineq3_min_v_func_decref(void);
casadi_int cineq3_min_v_func_n_in(void);
casadi_int cineq3_min_v_func_n_out(void);
casadi_real cineq3_min_v_func_default_in(casadi_int i);
const char* cineq3_min_v_func_name_in(casadi_int i);
const char* cineq3_min_v_func_name_out(casadi_int i);
const casadi_int* cineq3_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq3_min_v_func_sparsity_out(casadi_int i);
int cineq3_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq3_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq3_min_v_func_SZ_ARG 3
#define cineq3_min_v_func_SZ_RES 1
#define cineq3_min_v_func_SZ_IW 0
#define cineq3_min_v_func_SZ_W 1
int cineq4_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq4_max_v_func_alloc_mem(void);
int cineq4_max_v_func_init_mem(int mem);
void cineq4_max_v_func_free_mem(int mem);
int cineq4_max_v_func_checkout(void);
void cineq4_max_v_func_release(int mem);
void cineq4_max_v_func_incref(void);
void cineq4_max_v_func_decref(void);
casadi_int cineq4_max_v_func_n_in(void);
casadi_int cineq4_max_v_func_n_out(void);
casadi_real cineq4_max_v_func_default_in(casadi_int i);
const char* cineq4_max_v_func_name_in(casadi_int i);
const char* cineq4_max_v_func_name_out(casadi_int i);
const casadi_int* cineq4_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq4_max_v_func_sparsity_out(casadi_int i);
int cineq4_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq4_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq4_max_v_func_SZ_ARG 2
#define cineq4_max_v_func_SZ_RES 1
#define cineq4_max_v_func_SZ_IW 0
#define cineq4_max_v_func_SZ_W 1
int cineq4_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq4_min_v_func_alloc_mem(void);
int cineq4_min_v_func_init_mem(int mem);
void cineq4_min_v_func_free_mem(int mem);
int cineq4_min_v_func_checkout(void);
void cineq4_min_v_func_release(int mem);
void cineq4_min_v_func_incref(void);
void cineq4_min_v_func_decref(void);
casadi_int cineq4_min_v_func_n_in(void);
casadi_int cineq4_min_v_func_n_out(void);
casadi_real cineq4_min_v_func_default_in(casadi_int i);
const char* cineq4_min_v_func_name_in(casadi_int i);
const char* cineq4_min_v_func_name_out(casadi_int i);
const casadi_int* cineq4_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq4_min_v_func_sparsity_out(casadi_int i);
int cineq4_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq4_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq4_min_v_func_SZ_ARG 2
#define cineq4_min_v_func_SZ_RES 1
#define cineq4_min_v_func_SZ_IW 0
#define cineq4_min_v_func_SZ_W 1
int cineq5_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq5_max_v_func_alloc_mem(void);
int cineq5_max_v_func_init_mem(int mem);
void cineq5_max_v_func_free_mem(int mem);
int cineq5_max_v_func_checkout(void);
void cineq5_max_v_func_release(int mem);
void cineq5_max_v_func_incref(void);
void cineq5_max_v_func_decref(void);
casadi_int cineq5_max_v_func_n_in(void);
casadi_int cineq5_max_v_func_n_out(void);
casadi_real cineq5_max_v_func_default_in(casadi_int i);
const char* cineq5_max_v_func_name_in(casadi_int i);
const char* cineq5_max_v_func_name_out(casadi_int i);
const casadi_int* cineq5_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq5_max_v_func_sparsity_out(casadi_int i);
int cineq5_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq5_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq5_max_v_func_SZ_ARG 2
#define cineq5_max_v_func_SZ_RES 1
#define cineq5_max_v_func_SZ_IW 0
#define cineq5_max_v_func_SZ_W 1
int cineq5_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq5_min_v_func_alloc_mem(void);
int cineq5_min_v_func_init_mem(int mem);
void cineq5_min_v_func_free_mem(int mem);
int cineq5_min_v_func_checkout(void);
void cineq5_min_v_func_release(int mem);
void cineq5_min_v_func_incref(void);
void cineq5_min_v_func_decref(void);
casadi_int cineq5_min_v_func_n_in(void);
casadi_int cineq5_min_v_func_n_out(void);
casadi_real cineq5_min_v_func_default_in(casadi_int i);
const char* cineq5_min_v_func_name_in(casadi_int i);
const char* cineq5_min_v_func_name_out(casadi_int i);
const casadi_int* cineq5_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq5_min_v_func_sparsity_out(casadi_int i);
int cineq5_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq5_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq5_min_v_func_SZ_ARG 2
#define cineq5_min_v_func_SZ_RES 1
#define cineq5_min_v_func_SZ_IW 0
#define cineq5_min_v_func_SZ_W 1
int cineq6_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq6_max_v_func_alloc_mem(void);
int cineq6_max_v_func_init_mem(int mem);
void cineq6_max_v_func_free_mem(int mem);
int cineq6_max_v_func_checkout(void);
void cineq6_max_v_func_release(int mem);
void cineq6_max_v_func_incref(void);
void cineq6_max_v_func_decref(void);
casadi_int cineq6_max_v_func_n_in(void);
casadi_int cineq6_max_v_func_n_out(void);
casadi_real cineq6_max_v_func_default_in(casadi_int i);
const char* cineq6_max_v_func_name_in(casadi_int i);
const char* cineq6_max_v_func_name_out(casadi_int i);
const casadi_int* cineq6_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq6_max_v_func_sparsity_out(casadi_int i);
int cineq6_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq6_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq6_max_v_func_SZ_ARG 6
#define cineq6_max_v_func_SZ_RES 1
#define cineq6_max_v_func_SZ_IW 0
#define cineq6_max_v_func_SZ_W 1
int cineq6_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq6_min_v_func_alloc_mem(void);
int cineq6_min_v_func_init_mem(int mem);
void cineq6_min_v_func_free_mem(int mem);
int cineq6_min_v_func_checkout(void);
void cineq6_min_v_func_release(int mem);
void cineq6_min_v_func_incref(void);
void cineq6_min_v_func_decref(void);
casadi_int cineq6_min_v_func_n_in(void);
casadi_int cineq6_min_v_func_n_out(void);
casadi_real cineq6_min_v_func_default_in(casadi_int i);
const char* cineq6_min_v_func_name_in(casadi_int i);
const char* cineq6_min_v_func_name_out(casadi_int i);
const casadi_int* cineq6_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq6_min_v_func_sparsity_out(casadi_int i);
int cineq6_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq6_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq6_min_v_func_SZ_ARG 6
#define cineq6_min_v_func_SZ_RES 1
#define cineq6_min_v_func_SZ_IW 0
#define cineq6_min_v_func_SZ_W 1
int cineq7_max_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq7_max_v_func_alloc_mem(void);
int cineq7_max_v_func_init_mem(int mem);
void cineq7_max_v_func_free_mem(int mem);
int cineq7_max_v_func_checkout(void);
void cineq7_max_v_func_release(int mem);
void cineq7_max_v_func_incref(void);
void cineq7_max_v_func_decref(void);
casadi_int cineq7_max_v_func_n_in(void);
casadi_int cineq7_max_v_func_n_out(void);
casadi_real cineq7_max_v_func_default_in(casadi_int i);
const char* cineq7_max_v_func_name_in(casadi_int i);
const char* cineq7_max_v_func_name_out(casadi_int i);
const casadi_int* cineq7_max_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq7_max_v_func_sparsity_out(casadi_int i);
int cineq7_max_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq7_max_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq7_max_v_func_SZ_ARG 5
#define cineq7_max_v_func_SZ_RES 1
#define cineq7_max_v_func_SZ_IW 0
#define cineq7_max_v_func_SZ_W 8
int cineq7_min_v_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
int cineq7_min_v_func_alloc_mem(void);
int cineq7_min_v_func_init_mem(int mem);
void cineq7_min_v_func_free_mem(int mem);
int cineq7_min_v_func_checkout(void);
void cineq7_min_v_func_release(int mem);
void cineq7_min_v_func_incref(void);
void cineq7_min_v_func_decref(void);
casadi_int cineq7_min_v_func_n_in(void);
casadi_int cineq7_min_v_func_n_out(void);
casadi_real cineq7_min_v_func_default_in(casadi_int i);
const char* cineq7_min_v_func_name_in(casadi_int i);
const char* cineq7_min_v_func_name_out(casadi_int i);
const casadi_int* cineq7_min_v_func_sparsity_in(casadi_int i);
const casadi_int* cineq7_min_v_func_sparsity_out(casadi_int i);
int cineq7_min_v_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
int cineq7_min_v_func_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define cineq7_min_v_func_SZ_ARG 5
#define cineq7_min_v_func_SZ_RES 1
#define cineq7_min_v_func_SZ_IW 0
#define cineq7_min_v_func_SZ_W 8
#ifdef __cplusplus
} /* extern "C" */
#endif
