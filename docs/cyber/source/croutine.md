## croutine


# libco
libco协程库分析

## 目录
co_closure.h 定义了一些函数宏，具体的作用是什么？
co_comm.h 可重入锁，lockguard
co_epoll.h 网络接口
co_hook_sys_call.cpp hook系统调用
co_routine_inner.h
co_routine_specific.h


coctx_swap.S 栈切换
co_routine.h  
coctx.h  


## co_create
创建协程
```c++
int co_create( stCoRoutine_t **ppco,const stCoRoutineAttr_t *attr,pfn_co_routine_t pfn,void *arg )
{
	if( !co_get_curr_thread_env() ) 
	{
		co_init_curr_thread_env();
	}
	stCoRoutine_t *co = co_create_env( co_get_curr_thread_env(), attr, pfn,arg );
	*ppco = co;
	return 0;
}
```

## co_resume
```c++
void co_resume( stCoRoutine_t *co )
{
	stCoRoutineEnv_t *env = co->env;
	stCoRoutine_t *lpCurrRoutine = env->pCallStack[ env->iCallStackSize - 1 ];
	if( !co->cStart )
	{
		coctx_make( &co->ctx,(coctx_pfn_t)CoRoutineFunc,co,0 );
		co->cStart = 1;
	}
	env->pCallStack[ env->iCallStackSize++ ] = co;
	co_swap( lpCurrRoutine, co );


}
```

## co_yield
```c++
void co_yield_env( stCoRoutineEnv_t *env )
{
	
	stCoRoutine_t *last = env->pCallStack[ env->iCallStackSize - 2 ];
	stCoRoutine_t *curr = env->pCallStack[ env->iCallStackSize - 1 ];

	env->iCallStackSize--;

	co_swap( curr, last);
}
```

## co_release
```c++
void co_release( stCoRoutine_t *co )
{
    co_free( co );
}
```