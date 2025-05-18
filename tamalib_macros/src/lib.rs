//! Procedural macro for registering E0C6S46 instructions on Cpu methods.
//! Usage:
//! #[E0C6S46_instruction("PSET #0x%02X", 0xE40, MASK_7B, 0, 0, 5)]
//! fn op_pset(&mut self, arg1: u8, arg2: u8) { ... }

extern crate proc_macro;
use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, ItemFn, Expr, Token, ItemImpl};
use syn::punctuated::Punctuated;
use syn::parse::Parser;

#[proc_macro_attribute]
pub fn instruction(attr: TokenStream, item: TokenStream) -> TokenStream {
    // Parse the function and the attribute arguments as a comma-separated list of expressions
    let input_fn = parse_macro_input!(item as ItemFn);
    let args = Punctuated::<Expr, Token![,]>::parse_terminated.parse(attr).expect("Failed to parse attribute arguments");
    let exprs: Vec<_> = args.into_iter().collect();
    if exprs.len() != 6 {
        return syn::Error::new_spanned(&input_fn.sig.ident, "Expected 6 arguments").to_compile_error().into();
    }
    let log = &exprs[0];
    let code = &exprs[1];
    let mask = &exprs[2];
    let shift_arg0 = &exprs[3];
    let mask_arg0 = &exprs[4];
    let cycles = &exprs[5];
    let fn_name = &input_fn.sig.ident;

    let fn_name_str = fn_name.to_string();
    let const_ident = syn::Ident::new(&format!("INSTRUCTION_{}", fn_name_str.to_uppercase()), fn_name.span());

    let expanded = quote! {
        #input_fn
    };

    let const_block = quote! {
        #[allow(non_upper_case_globals)]
        const #const_ident: () = {
            inventory::submit! {
                crate::cpu::instructions::Instruction {
                    log: #log,
                    code: #code,
                    mask: #mask,
                    shift_arg0: #shift_arg0,
                    mask_arg0: #mask_arg0,
                    cycles: #cycles,
                    func: #fn_name,
                }
            }
        };
    };

    TokenStream::from(quote! {
        #const_block
        #expanded
        
    })
}

#[proc_macro_attribute]
pub fn register_instructions(_attr: TokenStream, item: TokenStream) -> TokenStream {
    let mut impl_block = parse_macro_input!(item as ItemImpl);
    let mut registrations = Vec::new();

    // Only works for inherent impls (not trait impls)
    let self_ty = &impl_block.self_ty;

    for item in impl_block.items.iter_mut() {
        if let syn::ImplItem::Fn(method) = item {
            let mut idx = None;
            for (i, attr) in method.attrs.iter().enumerate() {
                if attr.path().is_ident("instruction") {
                    idx = Some(i);
                    break;
                }
            }
            if let Some(attr_idx) = idx {
                let attr = method.attrs.remove(attr_idx);
                // Parse the attribute arguments
                let args = attr.parse_args_with(Punctuated::<Expr, Token![,]>::parse_terminated)
                    .expect("Failed to parse attribute arguments");
                let exprs: Vec<_> = args.into_iter().collect();
                if exprs.len() != 6 {
                    return syn::Error::new_spanned(&method.sig.ident, "Expected 6 arguments").to_compile_error().into();
                }
                let log = &exprs[0];
                let code = &exprs[1];
                let mask = &exprs[2];
                let shift_arg0 = &exprs[3];
                let mask_arg0 = &exprs[4];
                let cycles = &exprs[5];
                let fn_name = &method.sig.ident;
                // Register as <Type>::<fn_name>
                let registration = quote! {
                    inventory::submit! {
                        crate::cpu::instructions::Instruction {
                            log: #log,
                            code: #code,
                            mask: #mask,
                            shift_arg0: #shift_arg0,
                            mask_arg0: #mask_arg0,
                            cycles: #cycles,
                            func: #self_ty::#fn_name,
                        }
                    }
                };
                registrations.push(registration);
            }
        }
    }

    let expanded = quote! {
        #impl_block
        #(#registrations)*
    };
    TokenStream::from(expanded)
}
