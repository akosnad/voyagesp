#![no_std]

extern crate alloc;

pub mod at;

/// Newtype wrapper to skip null values when serializing JSON values.
#[derive(Debug)]
pub struct SkipNulls<'v>(pub &'v serde_json::Value);
impl<'v> serde::Serialize for SkipNulls<'v> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: serde::Serializer + ?Sized,
    {
        use serde::ser::{SerializeMap, SerializeSeq};

        match &self.0 {
            serde_json::Value::Object(map) => {
                let map = map.iter().filter(|(_, v)| !v.is_null());
                let mut ser = serializer.serialize_map(None)?;
                for (k, v) in map {
                    ser.serialize_entry(k, &SkipNulls(v))?;
                }
                ser.end()
            }
            serde_json::Value::Array(arr) => {
                let arr = arr.iter().filter(|v| !v.is_null());
                let mut ser = serializer.serialize_seq(None)?;
                for v in arr {
                    ser.serialize_element(&SkipNulls(v))?;
                }
                ser.end()
            }
            _ => self.0.serialize(serializer),
        }
    }
}
