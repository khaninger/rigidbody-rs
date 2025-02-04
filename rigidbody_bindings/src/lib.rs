use std::path::Path;
use std::os::raw::c_float;
use std::slice;

use rigidbody::multibody::Multibody; 
use rigidbody::Real;

#[no_mangle]
pub extern "C" fn multibody_new() -> *mut Multibody {
    //println!("We're doing it, man!");
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    Box::into_raw(Box::new(mb))
}

#[no_mangle]
pub extern "C" fn multibody_rnea<'a>(q_: *const Real,
                                     dq_: *const Real,
                                     ddq_: *const Real) -> *const Real {
    let q = unsafe{slice::from_raw_parts(q_, 7)};
    let dq = unsafe{slice::from_raw_parts(dq_, 7)};
    let ddq = unsafe{slice::from_raw_parts(ddq_, 7)};
                    
    //println!("Calling RNEA with\n    q:{:?}\n   dq:{:?}\n  ddq:{:?}", q, dq, ddq);
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    let ptr = Box::into_raw(Box::new(mb.rnea(q, dq, ddq)));

    ptr as *const Real
}

#[no_mangle]
pub extern "C" fn multibody_free(mb_ptr: *mut Multibody) {
    if !mb_ptr.is_null() {
        drop( unsafe { Box::from_raw(mb_ptr) } );
    }
}
