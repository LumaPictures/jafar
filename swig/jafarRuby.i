
#ifdef SWIG_VERSION > 0x010325
%trackobjects;
#endif

/* Gets the Ruby class which represents the klass C++ exception
 *   - module is the Ruby module name (for instance Kernel)
 *   - klass is the name (without namespace) of the exception class (KernelException)
 */
%define JAFAR_RUBY_EXCEPTION(module, klass)
%wrapper %{
VALUE klass ## _ruby_exception() {
    static VALUE klass_object = Qnil;
    VALUE module_object;
    ID klass_id;

    if (! NIL_P(klass_object))
        return klass_object;

    module_object = rb_define_module("Jafar");
    module_object = rb_define_module_under(module_object, #module);
    klass_id  = rb_intern("Rb" #klass);
    klass_object  = rb_const_get(module_object, klass_id);
    return klass_object;
}
%}
%enddef

/* A catch handler for a Jafar exception */
%define JAFAR_CATCH_MODULE_EXCEPTION(namespace, Klass)
    catch(jafar:: ## namespace ## :: ## Klass& e) {
        VALUE id_symbol = INT2NUM((int)e.getExceptionId());
        VALUE exception = rb_class_new_instance(1, &id_symbol, Klass ## _ruby_exception());
        rb_raise(exception, "%s", e.what().c_str());
    }
%enddef

%define JAFAR_CATCH_ROOT_EXCEPTION
    catch(jafar::kernel::Exception& e) {
        rb_raise(Exception_ruby_exception(), "%s", e.what().c_str());
    }
%enddef

%define JAFAR_CATCH_STD_EXCEPTION
    catch(std::exception& e)
    { rb_raise(rb_eRuntimeError, "%s", e.what()); }
%enddef

%define JAFAR_CATCH_ALL_EXCEPTIONS
    catch(...)
    { rb_raise(rb_eRuntimeError, "unknown error"); }
%enddef

%define JAFAR_EXCEPTION_SUPPORT
%header %{
    static ID id_to_symbol;
%}
%init %{
    id_to_symbol = rb_intern("id_to_symbol");
%}
JAFAR_RUBY_EXCEPTION(Kernel,     JafarException);
JAFAR_RUBY_EXCEPTION(Kernel,     Exception);
%enddef


%define JAFAR_CATCH_GENERIC
    JAFAR_CATCH_MODULE_EXCEPTION(kernel, JafarException)
    JAFAR_CATCH_ROOT_EXCEPTION
    JAFAR_CATCH_STD_EXCEPTION
    JAFAR_CATCH_ALL_EXCEPTIONS
%enddef

%define STATIC_VECTOR_TO_RUBY_ARRAY_RB_KLASS(vectorclassname, classname)
%typemap(out) vectorclassname &, const vectorclassname & {
 VALUE arr = rb_ary_new2($1->size());
 vectorclassname::iterator i = $1->begin(), iend = $1->end();
 for ( ; i!=iend; i++ )
 rb_ary_push(arr, Data_Wrap_Struct(classname, 0, 0, &*i));
 $result = arr;
}
%typemap(out) vectorclassname, const vectorclassname {
 VALUE arr = rb_ary_new2($1.size());
 vectorclassname::iterator i = $1.begin(), iend = $1.end();
 for ( ; i!=iend; i++ )
 rb_ary_push(arr, Data_Wrap_Struct(classname, 0, 0, &*i));
 $result = arr;
}
%enddef


%define STATIC_VECTOR_TO_RUBY_ARRAY(vectorclassname, classname)
STATIC_VECTOR_TO_RUBY_ARRAY_RB_KLASS(vectorclassname, c ## classname.klass )
%enddef

%define RUBY_ARRAY_TO_STATIC_VECTOR(vectorclassname, classname)
%typemap(in) vectorclassname, vectorclassname const, vectorclassname &, const vectorclassname &, vectorclassname const& {
 Check_Type($input, T_ARRAY);
 vectorclassname *vec = new vectorclassname;
 int len = RARRAY($input)->len;
 for (int i=0; i!=len; i++) {
 VALUE inst = rb_ary_entry($input, i);
 //The following _should_ work but doesn''t on HPUX
 Check_Type(inst, T_DATA);
 classname *element = NULL;
 Data_Get_Struct(inst, classname, element);
 vec->push_back(*element);
 }
 $1 = vec;
}
%enddef

%define RUBY_ARRAY_TO_PTR_VECTOR(vectorclassname, classname)
%typemap(in) vectorclassname, vectorclassname const, vectorclassname &, const vectorclassname &, vectorclassname const& {
 Check_Type($input, T_ARRAY);
 vectorclassname *vec = new vectorclassname;
 int len = RARRAY($input)->len;
 for (int i=0; i!=len; i++) {
 VALUE inst = rb_ary_entry($input, i);
 //The following _should_ work but doesn''t on HPUX
 // Check_Type(inst, T_DATA);
 classname *element = NULL;
 Data_Get_Struct(inst, classname, element);
 vec->push_back(new classname(*element));
 }
 $1 = vec;
}
%enddef
