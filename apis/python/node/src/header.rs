//! `dora.header`: the contract header of `dora_message::header`, from Python.
//!
//! Every rule is the Rust one; this module only converts. Values are checked
//! strictly on the way in: a bool is not an int and nothing is coerced to a
//! string, so a header Python accepts is one Rust would accept on the wire.

use dora_message::header::{self as core, Validity};
use dora_message::metadata::{MetadataParameters, Parameter};
use pyo3::create_exception;
use pyo3::exceptions::{PyTypeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyBool, PyDict, PyFloat, PyInt, PyString, PyTuple};

create_exception!(
    header,
    HeaderError,
    PyValueError,
    "A malformed or inconsistent contract header."
);

fn to_py(e: core::HeaderError) -> PyErr {
    HeaderError::new_err(e.message().to_string())
}

fn str_arg(name: &str, value: &Bound<'_, PyAny>) -> PyResult<String> {
    if !value.is_instance_of::<PyString>() {
        return Err(HeaderError::new_err(format!(
            "{name} must be a str, got {}",
            value.repr()?
        )));
    }
    value.extract()
}

fn int_arg(name: &str, value: &Bound<'_, PyAny>) -> PyResult<i64> {
    if value.is_instance_of::<PyBool>() || !value.is_instance_of::<PyInt>() {
        return Err(HeaderError::new_err(format!(
            "{name} must be an int, got {}",
            value.repr()?
        )));
    }
    value
        .extract()
        .map_err(|_| HeaderError::new_err(format!("{name} does not fit in a 64-bit signed int")))
}

fn validity_arg(value: &str) -> PyResult<Validity> {
    Validity::parse(value).map_err(to_py)
}

/// `Header`'s `validity` argument. Omitted, it is ok; an explicit `None` is
/// refused like any other non-str, never read as ok.
struct ValidityArg(Validity);

impl<'a, 'py> FromPyObject<'a, 'py> for ValidityArg {
    type Error = PyErr;

    fn extract(obj: Borrowed<'a, 'py, PyAny>) -> PyResult<Self> {
        validity_arg(&str_arg("validity", &obj)?).map(Self)
    }
}

/// One metadata value as the wire would carry it. Anything else is refused
/// rather than coerced, so `None` never becomes the string "None".
fn to_parameter(key: &str, value: &Bound<'_, PyAny>) -> PyResult<Parameter> {
    if value.is_instance_of::<PyBool>() {
        Ok(Parameter::Bool(value.extract()?))
    } else if value.is_instance_of::<PyInt>() {
        Ok(Parameter::Integer(int_arg(key, value)?))
    } else if value.is_instance_of::<PyFloat>() {
        Ok(Parameter::Float(value.extract()?))
    } else if value.is_instance_of::<PyString>() {
        Ok(Parameter::String(value.extract()?))
    } else {
        Err(HeaderError::new_err(format!(
            "metadata key {key:?} must be a string or an integer, got {}",
            value.repr()?
        )))
    }
}

#[pyclass(name = "Header", module = "dora.header", frozen, eq, from_py_object)]
#[derive(Clone, PartialEq)]
pub struct PyHeader(pub core::Header);

#[pymethods]
impl PyHeader {
    #[new]
    #[pyo3(
        signature = (frame_id, seq, stamp_ns, source, validity=ValidityArg(Validity::Ok), reason=None, child_frame=None),
        text_signature = "(frame_id, seq, stamp_ns, source, validity='ok', reason=None, child_frame=None)"
    )]
    fn new(
        frame_id: &Bound<'_, PyAny>,
        seq: &Bound<'_, PyAny>,
        stamp_ns: &Bound<'_, PyAny>,
        source: &Bound<'_, PyAny>,
        validity: ValidityArg,
        reason: Option<&Bound<'_, PyAny>>,
        child_frame: Option<&Bound<'_, PyAny>>,
    ) -> PyResult<Self> {
        let reason = reason.map(|r| str_arg("reason", r)).transpose()?;
        let header = core::Header::new(
            str_arg("frame_id", frame_id)?,
            int_arg("seq", seq)?,
            int_arg("stamp_ns", stamp_ns)?,
            str_arg("source", source)?,
            validity.0,
            reason,
        )
        .map_err(to_py)?;
        match child_frame {
            Some(c) => header
                .with_child_frame(str_arg("child_frame", c)?)
                .map(Self)
                .map_err(to_py),
            None => Ok(Self(header)),
        }
    }

    /// Read a header out of a metadata dict (`Header::read`). Other keys are ignored.
    #[staticmethod]
    fn from_metadata(metadata: &Bound<'_, PyAny>) -> PyResult<Self> {
        let dict = metadata.cast::<PyDict>().map_err(|_| {
            HeaderError::new_err(format!(
                "metadata must be a dict, got {}",
                metadata
                    .get_type()
                    .name()
                    .map_or("?".into(), |n| n.to_string())
            ))
        })?;
        let mut params = MetadataParameters::new();
        for key in core::KEYS {
            if let Some(value) = dict.get_item(key)? {
                params.insert(key.to_string(), to_parameter(key, &value)?);
            }
        }
        core::Header::read(&params).map(Self).map_err(to_py)
    }

    /// The header as metadata entries (`Header::write`): str and int values only.
    fn to_metadata<'py>(&self, py: Python<'py>) -> PyResult<Bound<'py, PyDict>> {
        let mut params = MetadataParameters::new();
        self.0.write(&mut params);
        let dict = PyDict::new(py);
        for (key, value) in params {
            match value {
                Parameter::String(s) => dict.set_item(key, s)?,
                Parameter::Integer(i) => dict.set_item(key, i)?,
                other => unreachable!("Header::write wrote {other:?}"),
            }
        }
        Ok(dict)
    }

    #[getter]
    fn frame_id(&self) -> &str {
        self.0.frame_id()
    }
    #[getter]
    fn seq(&self) -> i64 {
        self.0.seq()
    }
    #[getter]
    fn stamp_ns(&self) -> i64 {
        self.0.stamp_ns()
    }
    #[getter]
    fn source(&self) -> &str {
        self.0.source()
    }
    #[getter]
    fn validity(&self) -> &'static str {
        self.0.validity().as_str()
    }
    #[getter]
    fn reason(&self) -> Option<&str> {
        self.0.reason()
    }
    #[getter]
    fn child_frame(&self) -> Option<&str> {
        self.0.child_frame()
    }

    fn __repr__(&self) -> String {
        let h = &self.0;
        let mut s = format!(
            "Header(frame_id={:?}, seq={}, stamp_ns={}, source={:?}, validity={:?}",
            h.frame_id(),
            h.seq(),
            h.stamp_ns(),
            h.source(),
            h.validity().as_str()
        );
        if let Some(r) = h.reason() {
            s.push_str(&format!(", reason={r:?}"));
        }
        if let Some(c) = h.child_frame() {
            s.push_str(&format!(", child_frame={c:?}"));
        }
        s.push(')');
        s
    }
}

/// A counted upgrade for [`derive`]: `count(reason)` is called once, only
/// when the derived header is returned.
#[pyclass(name = "Upgrade", module = "dora.header", frozen, skip_from_py_object)]
pub struct PyUpgrade {
    #[pyo3(get)]
    reason: String,
    #[pyo3(get)]
    count: Py<PyAny>,
}

#[pymethods]
impl PyUpgrade {
    #[new]
    fn new(reason: &Bound<'_, PyAny>, count: Bound<'_, PyAny>) -> PyResult<Self> {
        if !count.is_callable() {
            return Err(PyTypeError::new_err("Upgrade count must be callable"));
        }
        Ok(Self {
            reason: str_arg("reason", reason)?,
            count: count.unbind(),
        })
    }
}

/// `dora_message::header::derive`: the header of a message computed from `parents`.
#[pyfunction]
#[pyo3(signature = (parents, frame_id, seq, validity, reason=None, upgrade=None))]
fn derive(
    py: Python<'_>,
    parents: Vec<PyHeader>,
    frame_id: &Bound<'_, PyAny>,
    seq: &Bound<'_, PyAny>,
    validity: &Bound<'_, PyAny>,
    reason: Option<&Bound<'_, PyAny>>,
    upgrade: Option<PyRef<'_, PyUpgrade>>,
) -> PyResult<PyHeader> {
    let parents: Vec<core::Header> = parents.into_iter().map(|p| p.0).collect();
    let frame_id = str_arg("frame_id", frame_id)?;
    let seq = int_arg("seq", seq)?;
    let validity = validity_arg(&str_arg("validity", validity)?)?;
    let reason = reason.map(|r| str_arg("reason", r)).transpose()?;
    let mut callback_error: Option<PyErr> = None;
    let mut count = |r: &str| {
        if let Some(up) = &upgrade
            && let Err(e) = up.count.call1(py, (r,))
        {
            callback_error = Some(e);
        }
    };
    let up = upgrade.as_ref().map(|u| core::Upgrade {
        reason: &u.reason,
        count: &mut count,
    });
    let header = core::derive(&parents, &frame_id, seq, validity, reason, up).map_err(to_py)?;
    match callback_error {
        Some(e) => Err(e),
        None => Ok(PyHeader(header)),
    }
}

pub fn register(parent: &Bound<'_, PyModule>) -> PyResult<()> {
    let py = parent.py();
    let m = PyModule::new(py, "header")?;
    m.add_class::<PyHeader>()?;
    m.add_class::<PyUpgrade>()?;
    m.add_function(wrap_pyfunction!(derive, &m)?)?;
    m.add("HeaderError", py.get_type::<HeaderError>())?;
    m.add("FRAME_ID", core::FRAME_ID)?;
    m.add("HEADER_SEQ", core::HEADER_SEQ)?;
    m.add("STAMP_NS", core::STAMP_NS)?;
    m.add("SOURCE", core::SOURCE)?;
    // `VALIDITY` is the tuple of values, as in the reference envelope; the key is `VALIDITY_KEY`.
    m.add("VALIDITY_KEY", core::VALIDITY)?;
    m.add("REASON", core::REASON)?;
    m.add("CHILD_FRAME", core::CHILD_FRAME)?;
    m.add("KEYS", PyTuple::new(py, core::KEYS)?)?;
    m.add(
        "VALIDITY",
        PyTuple::new(py, Validity::ALL.map(Validity::as_str))?,
    )?;
    parent.add_submodule(&m)?;
    // `from dora import header` needs the submodule in sys.modules.
    py.import("sys")?
        .getattr("modules")?
        .set_item("dora.header", &m)?;
    Ok(())
}
