use std::path::Path;
use std::os::raw::c_float;
use std::slice;

use rigidbody::multibody::Multibody; 
use rigidbody::Real;

#[no_mangle]
pub extern "C" fn multibody_new() -> *mut Multibody {
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    Box::into_raw(Box::new(mb))
}


#[no_mangle]
pub unsafe extern "C" fn multibody_rnea<'a>(mb_ptr: *const Multibody,
                                            q_: *const Real,
                                            dq_: *const Real,
                                            ddq_: *const Real
) -> *const Real {
  
    let mb = mb_ptr.as_ref().unwrap();
    let q = slice::from_raw_parts(q_, 7);
    let dq = slice::from_raw_parts(dq_, 7);
    let ddq = slice::from_raw_parts(ddq_, 7);
   
    let ptr = Box::into_raw(Box::new(mb.rnea(q, dq, ddq)));
    ptr as *const Real
}

#[no_mangle]
pub unsafe extern "C" fn multibody_crba<'a>(mb_ptr: *const Multibody,
                                            q_: *const Real,
) -> *mut [Real] {
  
    let mb = mb_ptr.as_ref().unwrap();
    let q = slice::from_raw_parts(q_, 7);

    let H = mb.crba(q);
    let flat_H: [Real; 49] = H.iter().cloned().collect::<Vec<_>>().try_into().expect("Convert");
    let boxed_H = Box::new(flat_H);
    let ptr = Box::into_raw(boxed_H);
    ptr
}


#[no_mangle]
pub unsafe extern "C" fn multibody_fwd_kin<'a>(mb_ptr: *const Multibody,
                                               q_: *const Real
) -> *const Real {  
    let mb = mb_ptr.as_ref().unwrap();
    let q = slice::from_raw_parts(q_, 7);

    let tr = mb.fwd_kin(q);
    let ptr = Box::into_raw(Box::new(tr.translation.vector));
    ptr as *const Real
}

#[no_mangle]
pub extern "C" fn multibody_free(mb_ptr: *mut Multibody) {
    if !mb_ptr.is_null() {
        drop( unsafe { Box::from_raw(mb_ptr) } );
    }
}
