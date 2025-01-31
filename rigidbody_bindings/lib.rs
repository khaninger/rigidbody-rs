use std::path::Path;

use rigidbody::multibody::Multibody; 

#[no_mangle]
pub extern "C" fn multibody_new() -> *mut Multibody {
    println!("We're doing it, man!");
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    Box::into_raw(Box::new(mb))
}

#[no_mangle]
pub extern "C" fn multibody_rnea<'a>(q: &'a [f32; 7],
                                     dq: &'a [f32; 7],
                                     ddq: &'a [f32; 7]) -> [f32; 7] {
    println!("Calling RNEA with\n    q:{:?}\n   dq:{:?}\n  ddq:{:?}", q, dq, ddq);
    let mb = Multibody::from_urdf(&Path::new("/home/hanikevi/rigidbody-rs/assets/fr3.urdf"));
    mb.rnea(q, dq, dq)
}

#[no_mangle]
pub extern "C" fn multibody_free(mb_ptr: *mut Multibody) {
    if !mb_ptr.is_null() {
        drop( unsafe { Box::from_raw(mb_ptr) } );
    }
}
