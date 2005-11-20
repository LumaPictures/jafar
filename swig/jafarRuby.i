
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

