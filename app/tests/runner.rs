use app::goertzel::goertzel;

#[test]
fn test_dc_values() {
    let x : [u16;20] = [4096;20];
    assert_eq!(1.0, goertzel(&x,6,12));
}

