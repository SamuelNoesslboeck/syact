use core::str::FromStr;

use proc_macro2::TokenStream;
use syn::DeriveInput;

fn sync_comp_group_impl(ast : DeriveInput) -> proc_macro::TokenStream {
    match ast.data {
        syn::Data::Struct(data) => {
            let name = ast.ident;
            let fields = data.fields;
            let fields_count = fields.len();

            let mut setup_stream = TokenStream::new();
            let mut index_stream = TokenStream::new();
            let mut index_stream_mut = TokenStream::new();

            let mut f_index : usize  = 0;
            for field in fields {
                let field_name = field.ident;
                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { &self.#n }
                } else {
                    TokenStream::from_str(&format!("&self.{}", f_index)).unwrap()
                };

                index_stream.extend::<TokenStream>(
                    quote::quote! {
                        if index == #f_index {
                            return #field_stream;
                        }
                    }.into()
                );

                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { &mut self.#n }
                } else {
                    TokenStream::from_str(&format!("&mut self.{}", f_index)).unwrap()
                };

                index_stream_mut.extend::<TokenStream>(
                    quote::quote! {
                        if index == #f_index {
                            return #field_stream;
                        }
                    }.into()
                );

                let field_stream = if let Some(n) = field_name.clone() {
                    quote::quote! { self.#n }
                } else {
                    TokenStream::from_str(&format!("self.{}", f_index)).unwrap()
                };

                setup_stream.extend::<TokenStream>(
                    quote::quote! {
                        #field_stream.setup()?;
                    }.into()
                );

                f_index += 1;
            }

            index_stream.extend::<TokenStream>(
                quote::quote! {
                    panic!("Index {} is out of bounds", index)
                }.into()
            );

            index_stream_mut.extend::<TokenStream>(
                quote::quote! {
                    panic!("Index {} is out of bounds", index)
                }.into()
            );

            quote::quote! {
                impl Setup for #name {
                    fn setup(&mut self) -> Result<(), stepper_lib::Error> {
                        #setup_stream
                        Ok(())
                    }
                }

                impl SyncCompGroup<#fields_count> for #name { 
                    fn index(&self, index : usize) -> &dyn SyncComp {
                        #index_stream
                    }

                    fn index_mut(&mut self, index : usize) -> &mut dyn SyncComp {
                        #index_stream_mut
                    }
                }
            }.into()
        },
        _ => panic!("This macro can only be used on structs")
    }
}

#[proc_macro_derive(SyncCompGroup)]
pub fn sync_comp_group_derive(input : proc_macro::TokenStream) -> proc_macro::TokenStream {
    let ast : DeriveInput = syn::parse(input).unwrap();
    sync_comp_group_impl(ast)
}